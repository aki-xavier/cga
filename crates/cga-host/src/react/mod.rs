//! React 运行时会话（真 React 19，在 boa 里跑）。
//!
//! 三层内置资产，加载顺序固定（scheduler 在模块初始化时探测宿主全局）：
//!
//! 1. `assets/react-shim.js` —— 平台层：确定性 `setTimeout`/microtask/`performance`/`console`。
//!    时间不由真实时钟推进，只由宿主显式 [`ReactSession::drain`] 推进，因此"挂载 → 提交 →
//!    副作用"整链可复现，不需要事件循环。
//! 2. `assets/react-runtime.{dev,prod}.js` —— 内置 React（scripts/vendor-react.mjs 生成）。
//! 3. `assets/react-host.js` —— 渲染器适配层：host config 把协调结果落成场景实例树，
//!    再快照成 `{t,p,c}`（与旧元素树同形，CSS/材质继承/`{__q}` 解析都不必改）。
//!
//! 会话语义：模块源码只求值一次（组件身份稳定 → hook 状态得以保留），根用一层固定的
//! `Root` 组件包裹；后续帧用 [`ReactSession::update`] 或组件内部 setState 驱动，React
//! 只重建变化的部分。
//!
//! # 与 JS 引擎的桥
//!
//! 所有对 JS 的接触都发生在这条引擎线程上（boa `Context` 是 `!Send`）。跨线程只传
//! `Send` 的命令闭包；返回值一律是**结构化信封字符串**（见 [`parse_envelope`]），由
//! JS 侧 `__sess.call` 生成——Rust 侧永远不接触引擎级异常对象。这是全项目唯一的
//! JS 引擎入口（`crates/cga-host/src/react/`）。
//!
//! - 统一下发：[`HostCall`] 是唯一的方法/参数编码点，JS 侧只有 `__sess.call` 一个入口。
//! - 结构化错误：[`ReactError`] 带 `kind`（syntax/runtime/internal），跨边界不丢类型。
//! - 帧事务：[`ReactSession::frame`] 一次做完挂载/输入/事件 + 重渲染 + drain + 快照。
//! - 契约版本：快照 `{t,p,c}` 的 schema 在建会话时由 [`SCENE_SCHEMA`] 断言。
//! - 沙箱：可选隔离模式，冻结宿主全局并清理作者新增全局（见 [`Sandbox`]）。
//!
//! # 线程模型
//!
//! 引擎跑在**专用线程**上（固定 64 MB 栈）：boa 的解析/求值是递归的，场景 JSX 嵌套越深
//! 需要的栈越大，debug 构建尤甚；把它隔离在自带大栈的线程里，调用方与测试都不必关心栈，
//! 也不必设 `RUST_MIN_STACK`。

use std::any::Any;
use std::cell::RefCell;
use std::collections::HashMap;
use std::marker::PhantomData;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::mpsc::{self, Sender};
use std::thread::JoinHandle;

use boa_engine::{Context, JsNativeError, JsValue, NativeFunction, Source};
use serde_json::{json, Value};

const SHIM: &str = include_str!("../../assets/react-shim.js");
const HOST: &str = include_str!("../../assets/react-host.js");
const RUNTIME_DEV: &str = include_str!("../../assets/react-runtime.dev.js");
const RUNTIME_PROD: &str = include_str!("../../assets/react-runtime.prod.js");

/// 引擎线程栈：足够深的中等场景 JSX（debug 构建下 8 MB 就会顶爆 `mechanical` 这种场景）。
const WORKER_STACK: usize = 64 << 20;

/// 场景快照 `{t,p,c}` 的契约版本。JS 侧 `__sess.schema()` 必须报告同一版本，否则建会话失败。
/// v1： `{t,p,c}`。v2：每个节点带 `__v`（自身提交版本）与 `__s`（子树版本）——
/// 增量构建的复用依据。
/// 升级 host 适配层时同步递增；渲染金标是第二道防线。
pub const SCENE_SCHEMA: u64 = 2;

/// 内置哪份 React 构建：`Prod`（默认，批量/CI）或 `Dev`（带 invalid hook call 等诊断）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Runtime {
    Dev,
    #[default]
    Prod,
}

impl Runtime {
    /// `CGA_REACT_DEV=1` 时用开发构建。
    pub fn from_env() -> Runtime {
        match std::env::var("CGA_REACT_DEV") {
            Ok(v) if !v.is_empty() && v != "0" => Runtime::Dev,
            _ => Runtime::Prod,
        }
    }

    fn source(self) -> &'static str {
        match self {
            Runtime::Dev => RUNTIME_DEV,
            Runtime::Prod => RUNTIME_PROD,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Runtime::Dev => "dev",
            Runtime::Prod => "prod",
        }
    }
}

/// 建会话选项。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SessionOptions {
    /// 沙箱模式：冻结宿主全局 + 每帧清理作者新增的全局。默认关闭。
    pub sandbox: Sandbox,
}

/// 会话池与长驻会话的类型标记：池化会话（[`Pooled`]）只能由线程本地池创建与复用；
/// 长驻交互会话（[`Owned`]）归调用方所有，不会被误塞回池里。
pub struct Owned;
pub struct Pooled;

/// 沙箱隔离策略说明（行为见 `assets/react-host.js`）。
///
/// - [`Sandbox::Off`]：作者代码可读写 `globalThis`（默认，交互测试依赖此行为）。
/// - [`Sandbox::On`]：模块求值前冻结宿主能力全局（`__sess`/`React`/`CGA_*`/`console`…），
///   并在每帧后删除作者新增的全局。这是**纵深防御**而非硬边界：单帧内作者仍能读到
///   未冻结的宿主对象；要彻底隔离需要独立 realm（见 `docs/jsx-css-host.md`）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Sandbox {
    /// 不隔离（默认）。
    #[default]
    Off,
    /// 冻结宿主全局 + 清理作者全局。
    On,
}

impl From<bool> for Sandbox {
    fn from(on: bool) -> Self {
        if on {
            Sandbox::On
        } else {
            Sandbox::Off
        }
    }
}

impl Sandbox {
    /// `CGA_SANDBOX=1` 时默认开启沙箱（与 `CGA_REACT_DEV` 一致的进程级开关）。
    pub fn from_env() -> Sandbox {
        match std::env::var("CGA_SANDBOX") {
            Ok(v) if !v.is_empty() && v != "0" => Sandbox::On,
            _ => Sandbox::Off,
        }
    }
}

/// 跨语言错误的类别，帮助宿主分流处理（不再只看错误字符串前缀）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ErrorKind {
    /// 模块源码编译失败（`new Function` 抛 SyntaxError）。
    Syntax,
    /// 渲染期/作者代码运行时异常。
    Runtime,
    /// 宿主自身错误（未知方法、参数编码失败、schema 不符等）。
    Internal,
}

impl ErrorKind {
    fn parse(s: &str) -> ErrorKind {
        match s {
            "syntax" => ErrorKind::Syntax,
            "internal" => ErrorKind::Internal,
            _ => ErrorKind::Runtime,
        }
    }
}

/// 结构化的跨语言错误。`kind` 在 `From<ReactError> for String` 时会被丢弃，但经过
/// [`ReactSession::begin`] / [`ReactSession::frame`] 的调用方可以直接读它。
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReactError {
    pub kind: ErrorKind,
    pub message: String,
    pub stack: String,
}

impl ReactError {
    pub(crate) fn internal(message: impl Into<String>) -> Self {
        ReactError {
            kind: ErrorKind::Internal,
            message: message.into(),
            stack: String::new(),
        }
    }
}

impl std::fmt::Display for ReactError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl std::error::Error for ReactError {}

impl From<ReactError> for String {
    fn from(e: ReactError) -> String {
        e.message
    }
}

/// 解构 JS 侧统一信封：`{ok:true,value}` / `{ok:false,kind,message,stack}`。
fn parse_envelope(s: &str) -> Result<Value, ReactError> {
    let v: Value = serde_json::from_str(s)
        .map_err(|e| ReactError::internal(format!("react: 信封解析失败: {e}")))?;
    if v.get("ok").and_then(Value::as_bool).unwrap_or(false) {
        Ok(v.get("value").cloned().unwrap_or(Value::Null))
    } else {
        Err(ReactError {
            kind: ErrorKind::parse(v.get("kind").and_then(Value::as_str).unwrap_or("")),
            message: v
                .get("message")
                .and_then(Value::as_str)
                .unwrap_or("")
                .to_string(),
            stack: v
                .get("stack")
                .and_then(Value::as_str)
                .unwrap_or("")
                .to_string(),
        })
    }
}

/// 一次 [`ReactSession::drain`] 的结果。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DrainStats {
    pub rounds: u32,
    pub errors: u32,
}

/// 渲染器适配层的协调计数（测试用：证明"局部更新"）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counters {
    pub create: u64,
    pub text: u64,
    pub update: u64,
    pub remove: u64,
    pub insert: u64,
    pub mount: u64,
}

/// 已提交的宿主实例（宿主拾取 / 测试用）。
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InstanceInfo {
    pub id: i64,
    pub type_name: String,
    /// 函数型 prop 名（事件处理器就在其中）。
    pub handlers: Vec<String>,
}

/// 一次事件派发的结果。
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DispatchOutcome {
    pub found: bool,
    pub id: i64,
    pub type_name: String,
    pub reason: String,
}

/// 一次帧事务的结果：快照 + 调度统计 + React 捕获的错误 + （可选）事件派发结果。
#[derive(Clone, Debug, PartialEq)]
pub struct FrameOutcome {
    /// 已提交实例树（`{t,p,c}` 的解析值；空场景为 `Value::Null`）。
    pub snapshot: Value,
    pub stats: DrainStats,
    /// React 捕获的错误（uncaught/caught/recoverable）。
    pub errors: Vec<String>,
    pub dispatch: Option<DispatchOutcome>,
}

/// 帧动作，对应 JS 侧 `frame({action})`。
pub enum FrameAction<'a> {
    /// 首帧：求值模块并以沙箱选项挂载。
    Mount {
        src: &'a str,
        pose: &'a HashMap<String, f64>,
        sandbox: Sandbox,
    },
    /// 推入宿主输入并重渲染。
    Input { json: &'a str },
    /// 派发事件并跑到静止。
    Event {
        id: i64,
        prop: &'a str,
        payload: &'a str,
    },
    /// 只跑到静止并快照。
    None,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LogEntry {
    pub level: String,
    pub text: String,
}

/// 宿主注册的原生函数签名（JS 可调用，例如 `solve`）。
pub type NativeFn = fn(&JsValue, &[JsValue], &mut Context) -> boa_engine::JsResult<JsValue>;

/// Rust → JS 的唯一调用编码点。新增宿主方法只需在这里加一个变体、在 JS 侧
/// `assets/react-host.js` 的 `HANDLERS` 表加一项——不再三处手写字符串调用。
#[derive(Clone, Debug)]
enum HostCall {
    Schema,
    Version,
    Begin {
        src: String,
        pose: HashMap<String, f64>,
        sandbox: bool,
    },
    Update,
    SetInput {
        json: String,
    },
    Drain {
        max_rounds: Option<u32>,
    },
    Snapshot,
    Ids,
    Counters,
    ResetCounters,
    Errors,
    Logs,
    Instances,
    Unmount,
    FrameMount {
        src: String,
        pose: HashMap<String, f64>,
        sandbox: bool,
    },
    FrameInput {
        json: String,
    },
    FrameEvent {
        id: i64,
        prop: String,
        payload: String,
    },
    FrameNone,
}

impl HostCall {
    fn to_json(&self) -> Value {
        match self {
            HostCall::Schema => json!({ "m": "schema" }),
            HostCall::Version => json!({ "m": "version" }),
            HostCall::Begin { src, pose, sandbox } => {
                json!({ "m": "begin", "a": { "src": src, "pose": pose, "sandbox": sandbox } })
            }
            HostCall::Update => json!({ "m": "update" }),
            HostCall::SetInput { json: input } => {
                json!({ "m": "setInput", "a": { "json": input } })
            }
            HostCall::Drain { max_rounds } => {
                json!({ "m": "drain", "a": { "maxRounds": max_rounds } })
            }
            HostCall::Snapshot => json!({ "m": "snapshot" }),
            HostCall::Ids => json!({ "m": "ids" }),
            HostCall::Counters => json!({ "m": "counters" }),
            HostCall::ResetCounters => json!({ "m": "resetCounters" }),
            HostCall::Errors => json!({ "m": "errors" }),
            HostCall::Logs => json!({ "m": "logs" }),
            HostCall::Instances => json!({ "m": "instances" }),
            HostCall::Unmount => json!({ "m": "unmount" }),
            HostCall::FrameMount { src, pose, sandbox } => json!({
                "m": "frame", "a": { "action": "mount", "src": src, "pose": pose, "sandbox": sandbox }
            }),
            HostCall::FrameInput { json: input } => {
                json!({ "m": "frame", "a": { "action": "input", "input": input } })
            }
            HostCall::FrameEvent { id, prop, payload } => json!({
                "m": "frame", "a": { "action": "event", "id": id, "prop": prop, "payload": payload }
            }),
            HostCall::FrameNone => json!({ "m": "frame", "a": { "action": "none" } }),
        }
    }
}

fn value_to_string(v: &Value) -> String {
    match v {
        Value::String(s) => s.clone(),
        other => other.to_string(),
    }
}

fn panic_message(payload: &(dyn Any + Send)) -> String {
    if let Some(s) = payload.downcast_ref::<&str>() {
        (*s).to_string()
    } else if let Some(s) = payload.downcast_ref::<String>() {
        s.clone()
    } else {
        "unknown panic".to_string()
    }
}

/// 引擎线程上持有的状态。
struct Worker {
    ctx: Context,
}

impl Worker {
    /// 载入内置资产（不做信封解析）。
    fn eval_raw(&mut self, code: &str) -> Result<String, String> {
        let v = self
            .ctx
            .eval(Source::from_bytes(code))
            .map_err(|e| format!("react: 载入资产失败: {e}"))?;
        let _ = self.ctx.run_jobs();
        v.to_string(&mut self.ctx)
            .map_err(|e| format!("react: 结果转字符串失败: {e}"))
            .map(|s| s.to_std_string_escaped())
    }

    /// 求值一段 JS，并解构结构化信封。
    fn eval_envelope(&mut self, code: &str) -> Result<Value, ReactError> {
        let v = self
            .ctx
            .eval(Source::from_bytes(code))
            .map_err(|e| ReactError::internal(format!("react: JS 求值失败: {e}")))?;
        let _ = self.ctx.run_jobs();
        let s = v
            .to_string(&mut self.ctx)
            .map_err(|e| ReactError::internal(format!("react: 结果转字符串失败: {e}")))?
            .to_std_string_escaped();
        parse_envelope(&s)
    }

    /// 求值任意表达式（`__cga_run` 包裹，测试/驱动用）。
    fn call_expr(&mut self, expr: &str) -> Result<Value, ReactError> {
        self.eval_envelope(&format!("__cga_run(() => {expr})"))
    }

    /// 求值任意表达式并取字符串值。
    fn call_str(&mut self, expr: &str) -> Result<String, ReactError> {
        self.call_expr(expr).map(|v| value_to_string(&v))
    }

    /// 下发一次 [`HostCall`]。
    fn rpc(&mut self, call: &HostCall) -> Result<Value, ReactError> {
        let payload = serde_json::to_string(&call.to_json())
            .map_err(|e| ReactError::internal(format!("react: 参数编码失败: {e}")))?;
        let literal = serde_json::to_string(&payload)
            .map_err(|e| ReactError::internal(format!("react: 参数转义失败: {e}")))?;
        self.eval_envelope(&format!("__sess.call({literal})"))
    }
}

type Job = Box<dyn FnOnce(&mut Worker) + Send>;

/// 一个 React 根 + 确定性调度器，跑在专用引擎线程上。整个会话常驻一个 boa `Context`。
pub struct ReactSession<M = Owned> {
    tx: Option<Sender<Job>>,
    join: Option<JoinHandle<()>>,
    runtime: Runtime,
    sandbox: Sandbox,
    _mode: PhantomData<M>,
}

impl ReactSession<Owned> {
    /// 建立会话：启动引擎线程，载入三层资产并创建 React 根。
    pub fn new(runtime: Runtime) -> Result<ReactSession<Owned>, String> {
        ReactSession::new_with(runtime, SessionOptions::default())
    }

    /// 同 [`ReactSession::new`]，但可指定 [`SessionOptions`]（沙箱模式）。
    pub fn new_with(runtime: Runtime, opts: SessionOptions) -> Result<ReactSession<Owned>, String> {
        Self::spawn(runtime, opts)
    }
}

impl ReactSession<Pooled> {
    /// 池化会话（仅由 [`with_session`] 使用）。
    fn new_pooled(runtime: Runtime) -> Result<ReactSession<Pooled>, String> {
        Self::spawn(runtime, SessionOptions::default())
    }
}

impl<M> ReactSession<M> {
    fn spawn(runtime: Runtime, opts: SessionOptions) -> Result<ReactSession<M>, String> {
        let (tx, rx) = mpsc::channel::<Job>();
        let (ready_tx, ready_rx) = mpsc::channel::<Result<(), String>>();
        let join = std::thread::Builder::new()
            .stack_size(WORKER_STACK)
            .name("cga-react".to_string())
            .spawn(move || {
                let mut w = Worker {
                    ctx: Context::default(),
                };
                let init = (|| -> Result<(), String> {
                    w.eval_raw(SHIM)?;
                    w.eval_raw(runtime.source())?;
                    w.eval_raw(HOST)?;
                    w.eval_raw(
                        "globalThis.__sess = CGA_REACT_HOST.createSession({ tag: 'concurrent' });",
                    )?;
                    Ok(())
                })();
                let ok = init.is_ok();
                let _ = ready_tx.send(init);
                if !ok {
                    return;
                }
                while let Ok(job) = rx.recv() {
                    job(&mut w);
                }
            })
            .map_err(|e| format!("react: 启动引擎线程失败: {e}"))?;
        ready_rx
            .recv()
            .map_err(|_| "react: 引擎线程未就绪".to_string())??;
        let sess = ReactSession {
            tx: Some(tx),
            join: Some(join),
            runtime,
            sandbox: opts.sandbox,
            _mode: PhantomData,
        };
        // 契约断言：JS 侧快照 schema 必须与 Rust 侧一致，避免两端 IR 静默漂移。
        let schema = sess
            .exec(|w| w.rpc(&HostCall::Schema))
            .map_err(String::from)?;
        if schema.as_u64() != Some(SCENE_SCHEMA) {
            return Err(format!(
                "react: 快照 schema 不符：Rust 期望 {SCENE_SCHEMA}，JS 报告 {}",
                value_to_string(&schema)
            ));
        }
        Ok(sess)
    }

    fn exec<T, F>(&self, f: F) -> Result<T, ReactError>
    where
        T: Send + 'static,
        F: FnOnce(&mut Worker) -> Result<T, ReactError> + Send + 'static,
    {
        let (reply_tx, reply_rx) = mpsc::channel();
        self.tx
            .as_ref()
            .ok_or_else(|| ReactError::internal("react: 会话已关闭"))?
            .send(Box::new(move |w| {
                let _ = reply_tx.send(f(w));
            }))
            .map_err(|_| ReactError::internal("react: 引擎线程已退出"))?;
        reply_rx
            .recv()
            .map_err(|_| ReactError::internal("react: 引擎线程未响应"))?
    }

    pub fn runtime(&self) -> Runtime {
        self.runtime
    }

    /// 沙箱模式。
    pub fn sandbox(&self) -> Sandbox {
        self.sandbox
    }

    /// 内置 React 版本。
    pub fn version(&mut self) -> Result<String, ReactError> {
        self.exec(|w| w.rpc(&HostCall::Version).map(|v| value_to_string(&v)))
    }

    /// 快照 `{t,p,c}` 的 schema 版本（建会话时已断言）。
    pub fn schema(&mut self) -> Result<u64, ReactError> {
        let v = self.exec(|w| w.rpc(&HostCall::Schema))?;
        Ok(v.as_u64().unwrap_or(0))
    }

    /// 推入宿主输入（props 监听）。设值后需 [`ReactSession::update`] 才会生效；
    /// 只有 `useContext(HostInput)` 的组件会重渲染，其余子树 bailout。
    pub fn set_input(&mut self, json: &str) -> Result<(), ReactError> {
        let call = HostCall::SetInput {
            json: json.to_string(),
        };
        self.exec(move |w| w.rpc(&call).map(|_| ()))
    }

    /// 推入输入 + 重渲染 + 跑到静止（一帧，单次跨线程）。
    pub fn apply_input(&mut self, json: &str) -> Result<DrainStats, ReactError> {
        Ok(self.frame(FrameAction::Input { json })?.stats)
    }

    /// 派发事件给某个实例（沿祖先链找第一个 `prop` 处理器），跑到静止，返回结果。
    pub fn dispatch(
        &mut self,
        id: i64,
        prop: &str,
        payload_json: &str,
    ) -> Result<DispatchOutcome, ReactError> {
        let out = self.frame(FrameAction::Event {
            id,
            prop,
            payload: payload_json,
        })?;
        Ok(out.dispatch.unwrap_or_default())
    }

    /// 已提交实例清单。
    pub fn instances(&mut self) -> Result<Vec<InstanceInfo>, ReactError> {
        let v = self.exec(|w| w.rpc(&HostCall::Instances))?;
        let list = v.as_array().cloned().unwrap_or_default();
        Ok(list
            .iter()
            .map(|e| InstanceInfo {
                id: e.get("id").and_then(Value::as_i64).unwrap_or(0),
                type_name: e
                    .get("type")
                    .and_then(Value::as_str)
                    .unwrap_or("")
                    .to_string(),
                handlers: e
                    .get("handlers")
                    .and_then(Value::as_array)
                    .map(|a| {
                        a.iter()
                            .filter_map(|h| h.as_str().map(str::to_string))
                            .collect()
                    })
                    .unwrap_or_default(),
            })
            .collect())
    }

    /// 注册一个宿主原生函数（JSX 宿主的 `solve` 就走这里）。
    ///
    /// 原生函数里的 panic 不会跨 FFI 边界 unwind：这里用 `catch_unwind` 捕获并转成
    /// JS 异常。
    pub fn register_global(
        &mut self,
        name: &str,
        arity: usize,
        f: NativeFn,
    ) -> Result<(), ReactError> {
        let name = name.to_string();
        self.exec(move |w| {
            let native = NativeFunction::from_copy_closure(
                move |this: &JsValue, args: &[JsValue], ctx: &mut Context| match catch_unwind(
                    AssertUnwindSafe(|| f(this, args, ctx)),
                ) {
                    Ok(result) => result,
                    Err(payload) => Err(JsNativeError::error()
                        .with_message(format!(
                            "host native fn panicked: {}",
                            panic_message(payload.as_ref())
                        ))
                        .into()),
                },
            );
            w.ctx
                .register_global_callable(boa_engine::JsString::from(name.as_str()), arity, native)
                .map_err(|e| ReactError::internal(format!("react: 注册原生函数 {name} 失败: {e}")))
        })
    }

    /// 求值一次模块源码（约定末行产生 `const __scene = …`）并以 pose 渲染。
    /// 模块只求值一次：组件身份稳定，后续帧的 setState 才能保住状态。
    pub fn begin(
        &mut self,
        module_src: &str,
        pose: &HashMap<String, f64>,
    ) -> Result<(), ReactError> {
        let call = HostCall::Begin {
            src: module_src.to_string(),
            pose: pose.clone(),
            sandbox: matches!(self.sandbox, Sandbox::On),
        };
        self.exec(move |w| w.rpc(&call).map(|_| ()))
    }

    /// 重新渲染（同一根元素身份）。
    pub fn update(&mut self) -> Result<(), ReactError> {
        self.exec(|w| w.rpc(&HostCall::Update).map(|_| ()))
    }

    pub fn unmount(&mut self) -> Result<(), ReactError> {
        self.exec(|w| w.rpc(&HostCall::Unmount).map(|_| ()))
    }

    /// 把调度器跑到静止（微任务 + 定时器，带轮数上限）。
    pub fn drain(&mut self) -> Result<DrainStats, ReactError> {
        // 先跑掉 boa 自己的 promise 任务，再让 JS 调度器 drain。
        let v = self.exec(|w| {
            let _ = w.ctx.run_jobs();
            let out = w.rpc(&HostCall::Drain { max_rounds: None });
            let _ = w.ctx.run_jobs();
            out
        })?;
        Ok(drain_stats(&v))
    }

    /// 帧事务：动作 + 重渲染 + drain + 快照，只跨线程一次。
    pub fn frame(&mut self, action: FrameAction) -> Result<FrameOutcome, ReactError> {
        let call = match action {
            FrameAction::Mount { src, pose, sandbox } => HostCall::FrameMount {
                src: src.to_string(),
                pose: pose.clone(),
                sandbox: matches!(sandbox, Sandbox::On),
            },
            FrameAction::Input { json } => HostCall::FrameInput {
                json: json.to_string(),
            },
            FrameAction::Event { id, prop, payload } => HostCall::FrameEvent {
                id,
                prop: prop.to_string(),
                payload: payload.to_string(),
            },
            FrameAction::None => HostCall::FrameNone,
        };
        let v = self.exec(move |w| w.rpc(&call))?;
        Ok(FrameOutcome {
            snapshot: v.get("snapshot").cloned().unwrap_or(Value::Null),
            stats: drain_stats(v.get("stats").unwrap_or(&Value::Null)),
            errors: v
                .get("errors")
                .and_then(Value::as_array)
                .map(|a| {
                    a.iter()
                        .filter_map(|e| e.as_str().map(str::to_string))
                        .collect()
                })
                .unwrap_or_default(),
            dispatch: v.get("dispatch").and_then(dispatch_outcome),
        })
    }

    /// 已提交场景实例树的 `{t,p,c}` JSON（与旧元素树同形）。
    pub fn snapshot(&mut self) -> Result<String, ReactError> {
        self.exec(|w| w.rpc(&HostCall::Snapshot).map(|v| value_to_string(&v)))
    }

    /// 场景实例 id（按树序），用于判定"身份是否稳定"。
    pub fn ids(&mut self) -> Result<Vec<i64>, ReactError> {
        let v = self.exec(|w| w.rpc(&HostCall::Ids))?;
        Ok(v.as_array()
            .map(|a| a.iter().filter_map(Value::as_i64).collect())
            .unwrap_or_default())
    }

    pub fn counters(&mut self) -> Result<Counters, ReactError> {
        let v = self.exec(|w| w.rpc(&HostCall::Counters))?;
        let n = |k: &str| v.get(k).and_then(Value::as_u64).unwrap_or(0);
        Ok(Counters {
            create: n("create"),
            text: n("text"),
            update: n("update"),
            remove: n("remove"),
            insert: n("insert"),
            mount: n("mount"),
        })
    }

    pub fn reset_counters(&mut self) -> Result<(), ReactError> {
        self.exec(|w| w.rpc(&HostCall::ResetCounters).map(|_| ()))
    }

    /// React 捕获到的错误（uncaught/caught/recoverable）。
    pub fn errors(&mut self) -> Result<Vec<String>, ReactError> {
        let v = self.exec(|w| w.rpc(&HostCall::Errors))?;
        Ok(v.as_array()
            .map(|a| {
                a.iter()
                    .filter_map(|e| e.as_str().map(str::to_string))
                    .collect()
            })
            .unwrap_or_default())
    }

    /// 作者 `console.*` 与调度器异常日志。
    pub fn logs(&mut self) -> Result<Vec<LogEntry>, ReactError> {
        let v = self.exec(|w| w.rpc(&HostCall::Logs))?;
        let list = v.as_array().cloned().unwrap_or_default();
        Ok(list
            .iter()
            .map(|e| LogEntry {
                level: e
                    .get("level")
                    .and_then(Value::as_str)
                    .unwrap_or("")
                    .to_string(),
                text: e
                    .get("text")
                    .and_then(Value::as_str)
                    .unwrap_or("")
                    .to_string(),
            })
            .collect())
    }

    /// 在会话里求值一段 JS 表达式（测试/驱动用），返回其字符串值。
    /// 异常（含语法错误）以 `Err` 返回。
    pub fn call(&mut self, expr: &str) -> Result<String, ReactError> {
        let expr = expr.to_string();
        self.exec(move |w| w.call_str(&expr))
    }
}

impl<M> Drop for ReactSession<M> {
    fn drop(&mut self) {
        self.tx = None; // 关闭通道 → 引擎线程结束
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

fn drain_stats(v: &Value) -> DrainStats {
    DrainStats {
        rounds: v.get("rounds").and_then(Value::as_u64).unwrap_or(0) as u32,
        errors: v.get("errs").and_then(Value::as_u64).unwrap_or(0) as u32,
    }
}

fn dispatch_outcome(v: &Value) -> Option<DispatchOutcome> {
    if v.is_null() {
        return None;
    }
    Some(DispatchOutcome {
        found: v.get("found").and_then(Value::as_bool).unwrap_or(false),
        id: v.get("id").and_then(Value::as_i64).unwrap_or(0),
        type_name: v
            .get("type")
            .and_then(Value::as_str)
            .unwrap_or("")
            .to_string(),
        reason: v
            .get("reason")
            .and_then(Value::as_str)
            .unwrap_or("")
            .to_string(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pose(pairs: &[(&str, f64)]) -> HashMap<String, f64> {
        pairs.iter().map(|(k, v)| (k.to_string(), *v)).collect()
    }

    /// 一段覆盖 hooks / 局部更新 / keyed 列表 / 副作用 / 卸载的模块。
    const MODULE: &str = r#"
globalThis.__effects = [];
const { useState, useEffect, useMemo, useReducer, useContext, memo, createContext } = React;
const Ctx = createContext('ctx-default');

const Child = memo(function Child({ n }) {
  const [x, setX] = useState(0);
  const label = useMemo(() => 'memo:' + n, [n]);
  const ctx = useContext(Ctx);
  const [red, dispatch] = useReducer((s, a) => s + a, 100);
  globalThis.bumpChild = () => setX((v) => v + 1);
  globalThis.dispatchChild = () => dispatch(5);
  return h('box', { id: 'child', x, label, ctx, red });
});

const App = function App() {
  const [count, setCount] = useState(0);
  const [items, setItems] = useState(['a', 'b']);
  globalThis.bumpApp = () => setCount((c) => c + 1);
  globalThis.addItem = () => setItems((a) => a.concat('i' + a.length));
  useEffect(() => {
    globalThis.__effects.push('effect:' + count);
    return () => globalThis.__effects.push('cleanup:' + count);
  }, [count]);
  return h('group', { id: 'app', count, pose: P.n },
    h('label', { text: 'count=' + count }),
    h(Child, { n: count }),
    items.map((it) => h('item', { key: it, name: it })));
};

const __scene = h(App, {});
"#;

    fn start() -> (ReactSession, serde_json::Value) {
        let mut s = ReactSession::new(Runtime::Prod).expect("session");
        s.begin(MODULE, &pose(&[("n", 3.0)])).expect("begin");
        let d = s.drain().expect("drain");
        assert_eq!(d.errors, 0, "drain 不该有异常");
        assert_eq!(s.errors().expect("errors"), Vec::<String>::new());
        let snap: serde_json::Value =
            serde_json::from_str(&s.snapshot().expect("snapshot")).unwrap();
        (s, snap)
    }

    fn snap(s: &mut ReactSession) -> serde_json::Value {
        serde_json::from_str(&s.snapshot().expect("snapshot")).unwrap()
    }

    fn effects(s: &mut ReactSession) -> String {
        s.call("JSON.stringify(globalThis.__effects)")
            .expect("effects")
    }

    #[test]
    fn reports_bundled_react_version() {
        let mut s = ReactSession::new(Runtime::Prod).unwrap();
        assert_eq!(s.version().unwrap(), "19.3.0");
        assert_eq!(s.schema().unwrap(), SCENE_SCHEMA);
        let mut d = ReactSession::new(Runtime::Dev).unwrap();
        assert_eq!(d.version().unwrap(), "19.3.0");
    }

    #[test]
    fn hooks_render_into_scene_instances() {
        let (mut s, snap) = start();
        assert_eq!(snap["t"], "group");
        assert_eq!(snap["p"]["count"], 0);
        assert_eq!(snap["p"]["pose"], 3, "pose P 注入到模块作用域");
        assert_eq!(snap["c"][0]["p"]["text"], "count=0");
        let child = &snap["c"][1];
        assert_eq!(child["t"], "box");
        assert_eq!(child["p"]["x"], 0, "useState");
        assert_eq!(child["p"]["label"], "memo:0", "useMemo");
        assert_eq!(child["p"]["ctx"], "ctx-default", "useContext");
        assert_eq!(child["p"]["red"], 100, "useReducer");
        assert_eq!(snap["c"][2]["p"]["name"], "a", "keyed 列表");
        assert_eq!(snap["c"][3]["p"]["name"], "b");
        assert_eq!(
            effects(&mut s),
            r#"["effect:0"]"#,
            "useEffect 在 drain 中执行"
        );
    }

    #[test]
    fn local_update_only_touches_affected_instance() {
        let (mut s, _) = start();
        let ids_before = s.ids().unwrap();
        s.reset_counters().unwrap();
        s.call("globalThis.bumpChild()").unwrap();
        s.drain().unwrap();
        let c = s.counters().unwrap();
        assert_eq!(c.create, 0, "局部更新不得新建实例");
        assert_eq!(c.update, 1, "只更新受影响的 1 个实例");
        assert_eq!(c.remove, 0);
        let snap = snap(&mut s);
        assert_eq!(snap["c"][1]["p"]["x"], 1, "子组件 state 生效");
        assert_eq!(snap["p"]["count"], 0, "父组件未受影响");
        assert_eq!(s.ids().unwrap(), ids_before, "实例身份全部不变");
    }

    #[test]
    fn instance_versions_track_changes() {
        let (mut s, snap0) = start();
        let own = |snap: &serde_json::Value| snap["__v"].as_u64().expect("__v");
        let sub = |snap: &serde_json::Value| snap["__s"].as_u64().expect("__s");
        let child_sub = |snap: &serde_json::Value, i: usize| snap["c"][i]["__s"].as_u64().unwrap();
        assert!(own(&snap0) > 0, "每个节点都要有自身版本");
        assert!(sub(&snap0) >= own(&snap0), "子树版本 ≥ 自身版本");

        // 局部更新：受影响子树的 __s 抬升，未受影响的兄弟不变；根的 __s 抬升但 __v 不变。
        // 增量构建靠"祖先链 __v 全不变 + 自身 __s 不变"判定复用。
        let (label0, box0, item0) = (
            child_sub(&snap0, 0),
            child_sub(&snap0, 1),
            child_sub(&snap0, 2),
        );
        s.call("globalThis.bumpChild()").unwrap();
        s.drain().unwrap();
        let snap1 = snap(&mut s);
        assert_eq!(child_sub(&snap1, 0), label0, "未受影响的兄弟子树版本不变");
        assert!(child_sub(&snap1, 1) > box0, "变化的子树版本抬升");
        assert_eq!(child_sub(&snap1, 2), item0, "keyed 列表未受影响");
        assert!(sub(&snap1) > sub(&snap0), "根子树版本抬升");
        assert_eq!(own(&snap1), own(&snap0), "根自身版本不变（props 没动）");

        // 结构变更（keyed 插入）抬父节点自身版本：子节点数量/顺序是上下文的一部分。
        s.call("globalThis.addItem()").unwrap();
        s.drain().unwrap();
        let snap2 = snap(&mut s);
        assert!(
            own(&snap2) > own(&snap1),
            "子节点增删必须抬父节点自身版本（否则复用会按旧位置配对）"
        );
    }

    #[test]
    fn props_change_and_effect_cleanup_ordering() {
        let (mut s, _) = start();
        s.call("globalThis.bumpApp()").unwrap();
        s.drain().unwrap();
        let snap = snap(&mut s);
        assert_eq!(snap["p"]["count"], 1);
        assert_eq!(snap["c"][0]["p"]["text"], "count=1");
        assert_eq!(snap["c"][1]["p"]["label"], "memo:1", "memo 子件收到新依赖");
        assert_eq!(
            effects(&mut s),
            r#"["effect:0","cleanup:0","effect:1"]"#,
            "副作用先清理再重建"
        );
    }

    #[test]
    fn keyed_insert_keeps_existing_instances() {
        let (mut s, before) = start();
        let ids_before = s.ids().unwrap();
        let n_before = before["c"].as_array().unwrap().len();
        s.reset_counters().unwrap();
        s.call("globalThis.addItem()").unwrap();
        s.drain().unwrap();
        let snap = snap(&mut s);
        assert_eq!(
            snap["c"].as_array().unwrap().len(),
            n_before + 1,
            "多了一项"
        );
        assert_eq!(snap["c"][n_before]["p"]["name"], "i2", "新项挂在末尾");
        let ids_after = s.ids().unwrap();
        assert_eq!(
            &ids_after[..ids_before.len()],
            &ids_before[..],
            "旧实例身份不变"
        );
        let c = s.counters().unwrap();
        assert_eq!(c.create, 1, "插入只新建 1 个实例");
        assert_eq!(c.remove, 0);
    }

    #[test]
    fn reducer_dispatch_and_unmount() {
        let (mut s, _) = start();
        s.call("globalThis.dispatchChild()").unwrap();
        s.drain().unwrap();
        assert_eq!(snap(&mut s)["c"][1]["p"]["red"], 105);

        s.reset_counters().unwrap();
        s.unmount().unwrap();
        s.drain().unwrap();
        assert_eq!(s.snapshot().unwrap(), "null", "卸载后场景为空");
        assert_eq!(s.counters().unwrap().remove, 1, "根实例被移除");
        let e = effects(&mut s);
        assert!(e.contains("cleanup:0"), "卸载触发副作用清理: {e}");
        assert_eq!(s.errors().unwrap(), Vec::<String>::new());
    }

    #[test]
    fn same_module_is_deterministic() {
        let (_a, sa) = start();
        let (_b, sb) = start();
        assert_eq!(sa, sb, "同输入同输出");
    }

    #[test]
    fn error_boundary_catches_and_renders_fallback() {
        const M: &str = r#"
const { Component } = React;
class Boundary extends Component {
  constructor(p) { super(p); this.state = { err: null }; }
  static getDerivedStateFromError(e) { return { err: String(e && e.message ? e.message : e) }; }
  render() { return this.state.err ? h('box', { s: [1, 1, 1], id: 'fallback' }) : this.props.children; }
}
function Boom() { throw new Error('kaboom'); }
const __scene = h(Boundary, {}, h(Boom, {}));
"#;
        let mut s = ReactSession::new(Runtime::Prod).unwrap();
        s.begin(M, &pose(&[])).unwrap();
        s.drain().unwrap();
        let snap = snap(&mut s);
        assert_eq!(snap["t"], "box", "错误边界渲染 fallback");
        assert_eq!(snap["p"]["id"], "fallback");
        let errs = s.errors().expect("errors");
        assert!(
            errs.iter().any(|e| e.contains("kaboom")),
            "错误被宿主捕获: {errs:?}"
        );
    }

    #[test]
    fn author_errors_carry_kind() {
        let mut s = ReactSession::new(Runtime::Prod).unwrap();
        let e = s.begin("const __scene = ;", &pose(&[])).unwrap_err();
        assert_eq!(e.kind, ErrorKind::Syntax, "语法错误: {e:?}");
        let e = s.begin("throw new Error('boom')", &pose(&[])).unwrap_err();
        assert_eq!(e.kind, ErrorKind::Runtime, "运行时错误: {e:?}");
        assert!(e.message.contains("boom"), "{e:?}");
    }

    #[test]
    fn author_errors_reach_the_host() {
        let mut s = ReactSession::new(Runtime::Prod).unwrap();
        assert!(
            s.begin("const __scene = ;", &pose(&[])).is_err(),
            "语法错误"
        );
        assert!(
            s.begin("throw new Error('boom')", &pose(&[])).is_err(),
            "运行时错误"
        );
        s.begin(
            "console.log('hi'); const __scene = h('group', {});",
            &pose(&[]),
        )
        .unwrap();
        s.drain().unwrap();
        let logs = s.logs().unwrap();
        assert!(
            logs.iter().any(|l| l.level == "log" && l.text == "hi"),
            "{logs:?}"
        );
    }

    #[test]
    fn frame_transaction_mounts_and_snapshots_in_one_call() {
        let mut s = ReactSession::new(Runtime::Prod).unwrap();
        let out = s
            .frame(FrameAction::Mount {
                src: "const __scene = h('group', { id: 'g' });",
                pose: &pose(&[]),
                sandbox: Sandbox::Off,
            })
            .unwrap();
        assert_eq!(out.snapshot["t"], "group");
        assert_eq!(out.stats.errors, 0);
        assert!(out.dispatch.is_none());
    }

    #[test]
    fn native_fn_panic_is_caught_and_reported() {
        fn boom(_: &JsValue, _: &[JsValue], _: &mut Context) -> boa_engine::JsResult<JsValue> {
            panic!("intentional native panic");
        }
        let mut s = ReactSession::new(Runtime::Prod).unwrap();
        s.register_global("boom", 0, boom).unwrap();
        let e = s.call("boom()").unwrap_err();
        assert!(
            e.message.contains("panicked") && e.message.contains("intentional native panic"),
            "panic 被转为 JS 异常: {e:?}"
        );
    }

    #[test]
    fn sandbox_protects_host_globals_and_clears_author_globals() {
        let mut s = ReactSession::new_with(
            Runtime::Prod,
            SessionOptions {
                sandbox: Sandbox::On,
            },
        )
        .unwrap();
        s.begin(
            "globalThis.__author = 1; globalThis.leak = 'x'; const __scene = h('group', {});",
            &pose(&[]),
        )
        .unwrap();
        s.drain().unwrap();
        assert_eq!(s.call("typeof globalThis.__author").unwrap(), "undefined");
        assert_eq!(s.call("typeof globalThis.leak").unwrap(), "undefined");
        // 宿主全局被冻结：赋值静默失败，对象仍在。
        s.call("React = null").unwrap();
        assert_eq!(s.call("React === null").unwrap(), "false");
        assert_eq!(s.call("typeof __sess").unwrap(), "object");
    }

    #[test]
    fn own_author_global_helper() {
        // 非沙箱模式下作者全局按旧行为保留（交互测试依赖）。
        let mut s = ReactSession::new(Runtime::Prod).unwrap();
        s.begin(
            "globalThis.helper = 7; const __scene = h('group', {});",
            &pose(&[]),
        )
        .unwrap();
        assert_eq!(s.call("globalThis.helper").unwrap(), "7");
    }
}

thread_local! {
    /// 进程内会话池：把运行时解析（数百毫秒）从"每次渲染"降到"每个线程一次"。
    /// 只放 [`Pooled`] 会话——长驻交互会话是 [`Owned`]，永远不会被塞回这里。
    static POOL: RefCell<HashMap<Runtime, ReactSession<Pooled>>> = RefCell::new(HashMap::new());
}

/// 复用一个池化会话。到期归还，出错也归还（会话仍可用）。
/// 每次渲染的固定开销因此只剩"模块 JS 求值 + React 挂载/提交"。
pub fn with_session<T>(
    runtime: Runtime,
    f: impl FnOnce(&mut ReactSession<Pooled>) -> Result<T, String>,
) -> Result<T, String> {
    POOL.with(|cell| {
        let mut pool = cell.borrow_mut();
        let mut sess = match pool.remove(&runtime) {
            Some(s) => s,
            None => ReactSession::new_pooled(runtime)?,
        };
        let out = f(&mut sess);
        pool.insert(runtime, sess);
        out
    })
}
