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

use std::collections::HashMap;

use boa_engine::{Context, Source};

const SHIM: &str = include_str!("../../assets/react-shim.js");
const HOST: &str = include_str!("../../assets/react-host.js");
const RUNTIME_DEV: &str = include_str!("../../assets/react-runtime.dev.js");
const RUNTIME_PROD: &str = include_str!("../../assets/react-runtime.prod.js");

const OK: &str = "__CGA_OK__";
const ERR: &str = "__CGA_ERR__";

/// 内置哪份 React 构建：`Prod`（默认，批量/CI）或 `Dev`（带 invalid hook call 等诊断）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
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

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LogEntry {
    pub level: String,
    pub text: String,
}

/// 一个 React 根 + 确定性调度器。整个会话常驻一个 boa `Context`。
pub struct ReactSession {
    ctx: Context,
    runtime: Runtime,
}

impl ReactSession {
    /// 建立会话：载入三层资产并创建 React 根。
    pub fn new(runtime: Runtime) -> Result<ReactSession, String> {
        let mut s = ReactSession {
            ctx: Context::default(),
            runtime,
        };
        s.eval_raw(SHIM)?;
        s.eval_raw(runtime.source())?;
        s.eval_raw(HOST)?;
        s.eval_raw("globalThis.__sess = CGA_REACT_HOST.createSession({ tag: 'concurrent' });")?;
        Ok(s)
    }

    pub fn runtime(&self) -> Runtime {
        self.runtime
    }

    /// 内置 React 版本。
    pub fn version(&mut self) -> Result<String, String> {
        self.call("CGA_REACT_HOST.version")
    }

    /// 求值一次模块源码（约定末行产生 `const __scene = …`）并以 pose 渲染。
    /// 模块只求值一次：组件身份稳定，后续帧的 setState 才能保住状态。
    pub fn begin(&mut self, module_src: &str, pose: &HashMap<String, f64>) -> Result<(), String> {
        let src = serde_json::to_string(module_src).map_err(|e| e.to_string())?;
        let pose = serde_json::to_string(pose).map_err(|e| e.to_string())?;
        self.call(&format!("__sess.begin({src}, {pose})"))?;
        Ok(())
    }

    /// 重新渲染（同一根元素身份）。
    pub fn update(&mut self) -> Result<(), String> {
        self.call("__sess.update()")?;
        Ok(())
    }

    pub fn unmount(&mut self) -> Result<(), String> {
        self.call("__sess.unmount()")?;
        Ok(())
    }

    /// 把调度器跑到静止（微任务 + 定时器，带轮数上限）。
    pub fn drain(&mut self) -> Result<DrainStats, String> {
        let _ = self.ctx.run_jobs();
        let s = self.call("JSON.stringify(__sess.drain())")?;
        let v: serde_json::Value =
            serde_json::from_str(&s).map_err(|e| format!("react: drain parse: {e}"))?;
        let out = DrainStats {
            rounds: v.get("rounds").and_then(serde_json::Value::as_u64).unwrap_or(0) as u32,
            errors: v.get("errs").and_then(serde_json::Value::as_u64).unwrap_or(0) as u32,
        };
        let _ = self.ctx.run_jobs();
        Ok(out)
    }

    /// 已提交场景实例树的 `{t,p,c}` JSON（与旧元素树同形）。
    pub fn snapshot(&mut self) -> Result<String, String> {
        self.call("__sess.snapshot()")
    }

    /// 场景实例 id（按树序），用于判定"身份是否稳定"。
    pub fn ids(&mut self) -> Result<Vec<i64>, String> {
        let s = self.call("__sess.ids()")?;
        serde_json::from_str(&s).map_err(|e| format!("react: ids parse: {e}"))
    }

    pub fn counters(&mut self) -> Result<Counters, String> {
        let s = self.call("__sess.counters()")?;
        let v: serde_json::Value =
            serde_json::from_str(&s).map_err(|e| format!("react: counters parse: {e}"))?;
        let n = |k: &str| v.get(k).and_then(serde_json::Value::as_u64).unwrap_or(0);
        Ok(Counters {
            create: n("create"),
            text: n("text"),
            update: n("update"),
            remove: n("remove"),
            insert: n("insert"),
            mount: n("mount"),
        })
    }

    pub fn reset_counters(&mut self) -> Result<(), String> {
        self.call("__sess.resetCounters()")?;
        Ok(())
    }

    /// React 捕获到的错误（uncaught/caught/recoverable）。
    pub fn errors(&mut self) -> Result<Vec<String>, String> {
        let s = self.call("__sess.errors()")?;
        serde_json::from_str(&s).map_err(|e| format!("react: errors parse: {e}"))
    }

    /// 作者 `console.*` 与调度器异常日志。
    pub fn logs(&mut self) -> Result<Vec<LogEntry>, String> {
        let s = self.call("__sess.logs()")?;
        let v: Vec<serde_json::Value> =
            serde_json::from_str(&s).map_err(|e| format!("react: logs parse: {e}"))?;
        Ok(v.iter()
            .map(|e| LogEntry {
                level: e
                    .get("level")
                    .and_then(serde_json::Value::as_str)
                    .unwrap_or("")
                    .to_string(),
                text: e
                    .get("text")
                    .and_then(serde_json::Value::as_str)
                    .unwrap_or("")
                    .to_string(),
            })
            .collect())
    }

    /// 在会话里求值一段 JS 表达式（测试/驱动用），返回其字符串值。
    /// 异常（含语法错误）以 `Err` 返回。
    pub fn call(&mut self, expr: &str) -> Result<String, String> {
        self.eval(&format!("__cga_run(() => {expr})"))
    }

    /// 不求值包装、直接把源码交给引擎（仅用于载入我们的资产）。
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

    fn eval(&mut self, code: &str) -> Result<String, String> {
        let v = self
            .ctx
            .eval(Source::from_bytes(code))
            .map_err(|e| format!("react: JS 求值失败: {e}"))?;
        let _ = self.ctx.run_jobs();
        let s = v
            .to_string(&mut self.ctx)
            .map_err(|e| format!("react: 结果转字符串失败: {e}"))?
            .to_std_string_escaped();
        if let Some(rest) = s.strip_prefix(OK) {
            Ok(rest.to_string())
        } else if let Some(rest) = s.strip_prefix(ERR) {
            Err(rest.to_string())
        } else {
            Ok(s)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// boa 的解析/求值是递归的；debug 构建的栈帧大得多，854 KB 的 dev bundle 会顶爆
    /// 默认线程栈（release 下默认栈就够）。测试统一在 64 MB 栈上跑，自给自足，
    /// 不依赖 RUST_MIN_STACK 之类的环境变量。
    fn big_stack<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> T {
        std::thread::Builder::new()
            .stack_size(64 << 20)
            .spawn(f)
            .expect("spawn")
            .join()
            .expect("测试线程 panic")
    }

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
        s.call("JSON.stringify(globalThis.__effects)").expect("effects")
    }

    #[test]
    fn reports_bundled_react_version() {
        big_stack(|| {
            let mut s = ReactSession::new(Runtime::Prod).unwrap();
            assert_eq!(s.version().unwrap(), "19.3.0");
            let mut d = ReactSession::new(Runtime::Dev).unwrap();
            assert_eq!(d.version().unwrap(), "19.3.0");
        });
    }

    #[test]
    fn hooks_render_into_scene_instances() {
        big_stack(|| {
            let (mut s, snap) = start();
            assert_eq!(snap[0]["t"], "group");
            assert_eq!(snap[0]["p"]["count"], 0);
            assert_eq!(snap[0]["p"]["pose"], 3, "pose P 注入到模块作用域");
            assert_eq!(snap[0]["c"][0]["p"]["text"], "count=0");
            let child = &snap[0]["c"][1];
            assert_eq!(child["t"], "box");
            assert_eq!(child["p"]["x"], 0, "useState");
            assert_eq!(child["p"]["label"], "memo:0", "useMemo");
            assert_eq!(child["p"]["ctx"], "ctx-default", "useContext");
            assert_eq!(child["p"]["red"], 100, "useReducer");
            assert_eq!(snap[0]["c"][2]["p"]["name"], "a", "keyed 列表");
            assert_eq!(snap[0]["c"][3]["p"]["name"], "b");
            assert_eq!(effects(&mut s), r#"["effect:0"]"#, "useEffect 在 drain 中执行");
        });
    }

    #[test]
    fn local_update_only_touches_affected_instance() {
        big_stack(|| {
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
            assert_eq!(snap[0]["c"][1]["p"]["x"], 1, "子组件 state 生效");
            assert_eq!(snap[0]["p"]["count"], 0, "父组件未受影响");
            assert_eq!(s.ids().unwrap(), ids_before, "实例身份全部不变");
        });
    }

    #[test]
    fn props_change_and_effect_cleanup_ordering() {
        big_stack(|| {
            let (mut s, _) = start();
            s.call("globalThis.bumpApp()").unwrap();
            s.drain().unwrap();
            let snap = snap(&mut s);
            assert_eq!(snap[0]["p"]["count"], 1);
            assert_eq!(snap[0]["c"][0]["p"]["text"], "count=1");
            assert_eq!(snap[0]["c"][1]["p"]["label"], "memo:1", "memo 子件收到新依赖");
            assert_eq!(
                effects(&mut s),
                r#"["effect:0","cleanup:0","effect:1"]"#,
                "副作用先清理再重建"
            );
        });
    }

    #[test]
    fn keyed_insert_keeps_existing_instances() {
        big_stack(|| {
            let (mut s, before) = start();
            let ids_before = s.ids().unwrap();
            let n_before = before[0]["c"].as_array().unwrap().len();
            s.reset_counters().unwrap();
            s.call("globalThis.addItem()").unwrap();
            s.drain().unwrap();
            let snap = snap(&mut s);
            assert_eq!(snap[0]["c"].as_array().unwrap().len(), n_before + 1, "多了一项");
            assert_eq!(snap[0]["c"][n_before]["p"]["name"], "i2", "新项挂在末尾");
            let ids_after = s.ids().unwrap();
            assert_eq!(
                &ids_after[..ids_before.len()],
                &ids_before[..],
                "旧实例身份不变"
            );
            let c = s.counters().unwrap();
            assert_eq!(c.create, 1, "插入只新建 1 个实例");
            assert_eq!(c.remove, 0);
        });
    }

    #[test]
    fn reducer_dispatch_and_unmount() {
        big_stack(|| {
            let (mut s, _) = start();
            s.call("globalThis.dispatchChild()").unwrap();
            s.drain().unwrap();
            assert_eq!(snap(&mut s)[0]["c"][1]["p"]["red"], 105);

            s.reset_counters().unwrap();
            s.unmount().unwrap();
            s.drain().unwrap();
            assert_eq!(s.snapshot().unwrap(), "[]", "卸载后场景为空");
            assert_eq!(s.counters().unwrap().remove, 1, "根实例被移除");
            let e = effects(&mut s);
            assert!(e.contains("cleanup:0"), "卸载触发副作用清理: {e}");
            assert_eq!(s.errors().unwrap(), Vec::<String>::new());
        });
    }

    #[test]
    fn same_module_is_deterministic() {
        big_stack(|| {
            let (_a, sa) = start();
            let (_b, sb) = start();
            assert_eq!(sa, sb, "同输入同输出");
        });
    }

    #[test]
    fn author_errors_reach_the_host() {
        big_stack(|| {
            let mut s = ReactSession::new(Runtime::Prod).unwrap();
            assert!(s.begin("const __scene = ;", &pose(&[])).is_err(), "语法错误");
            assert!(
                s.begin("throw new Error('boom')", &pose(&[])).is_err(),
                "运行时错误"
            );
            s.begin("console.log('hi'); const __scene = h('group', {});", &pose(&[]))
                .unwrap();
            s.drain().unwrap();
            let logs = s.logs().unwrap();
            assert!(
                logs.iter().any(|l| l.level == "log" && l.text == "hi"),
                "{logs:?}"
            );
        });
    }
}
