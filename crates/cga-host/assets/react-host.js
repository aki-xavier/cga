// cga-react 渲染器适配层（我们自己的源码，不随 vendor 重新打包）。
//
// 把 react-reconciler 的协调结果落成一棵**场景实例树**（普通 JS 对象），宿主再把它
// 快照成 {t,p,c} JSON 交给 Rust 构建场景——与旧的元素树形状一致，于是 CSS 匹配、
// 材质继承、惰性查询解析（{__q}）全都不需要改。
//
// 已按 React 19（react-reconciler 0.34）核对：
//   - prepareUpdate 已移除，commitUpdate(instance, type, prevProps, nextProps) 自己 diff
//   - props.ref 是普通 prop（场景属性里要剥掉）
//   - Dev 构建额外要求性能追踪 / View Transition / test selector 那一组键
//
// 加载顺序：react-shim.js → react-runtime.*.js → react-host.js
//
// 与 Rust 的桥（见 src/react/mod.rs）：
//   - 唯一 RPC 入口 `__sess.call(payloadJson)`：payload = {"m": 方法名, "a": 参数对象}，
//     返回结构化信封字符串 {"ok":true,"value":…} / {"ok":false,"kind","message","stack"}。
//   - 帧事务 `frame({action})`：mount / input / event / none，内部一次做完
//     重渲染 + drain + 快照，避免多次跨线程往返。
//   - `schema()` 报告 `{t,p,c}` 快照契约版本，Rust 建会话时断言。
//   - sandbox：可选隔离——冻结宿主全局，并在每帧后清掉作者新增的全局。
(function () {
  'use strict';

  const R = globalThis.CGA_REACT;
  const S = globalThis.CGA_SCHED;
  if (!R || !S) {
    throw new Error('react-host: load react-shim.js and react-runtime.*.js first');
  }
  const React = R.React;

  // ---- DevTools 握手（E3）：渲染器侧契约的诚实最小实现 -------------------
  // react-reconciler 在创建 reconciler 时若发现 __REACT_DEVTOOLS_GLOBAL_HOOK__
  // 会 inject(renderer)，并在每次提交后调 hook.onCommitFiberRoot。这里装记录型桩：
  // 渲染器注册 + 提交计数可观测、可测试。完整的检查器协议（选择/高亮/props 编辑）
  // 是 DevTools 后端（浏览器扩展）的事——渲染器侧的义务就是这个握手。
  if (!globalThis.__REACT_DEVTOOLS_GLOBAL_HOOK__) {
    let nextRid = 1;
    globalThis.__REACT_DEVTOOLS_GLOBAL_HOOK__ = {
      renderers: new Map(),
      commits: [], // 每次提交 { rid, didError }
      supportsFiber: true,
      checkDCE() {},
      inject(renderer) {
        const id = nextRid++;
        this.renderers.set(id, renderer);
        return id;
      },
      onCommitFiberRoot(rid, _root, _priority, didError) {
        this.commits.push({ rid, didError: !!didError });
      },
      onCommitFiberUnmount() {},
      onScheduleFiberRoot() {},
      sub() {
        return 0;
      },
      unsub() {},
    };
  }

  // E2：动态 import() 的模块命名空间（bundler 把 import("./x.jsx") 改写成
  // __cga_dyn_import(id)；模块体在打包时已全部求值——Promise 只包命名空间）。
  globalThis.__cga_dyn_import = (id) => {
    const ns = globalThis['__exp_' + id];
    if (!ns) return Promise.reject(new Error('cga: no bundled module __exp_' + id));
    return Promise.resolve(ns);
  };

  // 场景源码可以像旧 PRELUDE 一样直接用 h()/Fragment（P1 的 PRELUDE 会覆盖为其增强版）。
  globalThis.React = React;
  globalThis.h = React.createElement;
  globalThis.Fragment = React.Fragment;

  // ---- 结构化信封（跨语言错误契约）---------------------------------------
  // 宿主只把字符串送回 Rust：成功 {ok:true,value}，失败 {ok:false,kind,message,stack}。
  // kind: 'syntax'（模块编译失败）/ 'runtime'（渲染或作者代码）/ 'internal'（宿主自身）。
  const errorInfo = (e) => {
    const message = e !== null && typeof e === 'object' && e.message !== undefined
      ? String(e.message)
      : String(e);
    const stack = e !== null && typeof e === 'object' && e.stack !== undefined ? String(e.stack) : '';
    let kind = 'runtime';
    if (e !== null && typeof e === 'object' && e.cgaKind) kind = String(e.cgaKind);
    else if (e !== null && typeof e === 'object' && e.name === 'SyntaxError') kind = 'syntax';
    return { kind, message, stack };
  };
  const envelope = (fn) => {
    try {
      return JSON.stringify({ ok: true, value: fn() });
    } catch (e) {
      return JSON.stringify(Object.assign({ ok: false }, errorInfo(e)));
    }
  };

  // ---- 沙箱：冻结宿主全局 + 每帧清理作者新增全局 --------------------------
  // 场景代码（可能是 LLM 生成的）在 new Function 里以全局作用域求值，能读写 globalThis。
  // 沙箱模式做两件事：宿主能力（__sess / React / CGA_*）在模块求值前变为不可写、不可配置；
  // 模块或渲染期间作者新增的全局，在每帧结束（drain 之后）被删除。注意这是纵深防御，
  // 不是硬边界（作者仍可在单帧内读到非冻结的全局）。
  const HOST_GLOBALS = [
    'CGA_REACT', 'CGA_SCHED', 'CGA_REACT_HOST', '__sess', '__cga_run',
    'React', 'h', 'Fragment', 'HostInput',
    'console', 'setTimeout', 'clearTimeout', 'queueMicrotask', 'performance',
  ];
  let sandboxBase = null;
  const protectGlobal = (name) => {
    const d = Object.getOwnPropertyDescriptor(globalThis, name);
    if (!d || !d.configurable || d.get || d.set) return;
    try {
      Object.defineProperty(globalThis, name, {
        value: d.value,
        writable: false,
        enumerable: d.enumerable,
        configurable: false,
      });
    } catch (_) {
      /* 已是不可配置或环境限制，忽略 */
    }
  };
  const sandboxBegin = () => {
    for (const name of HOST_GLOBALS) protectGlobal(name);
    sandboxBase = new Set(Object.getOwnPropertyNames(globalThis));
  };
  const sandboxSweep = () => {
    if (sandboxBase === null) return 0;
    let removed = 0;
    for (const name of Object.getOwnPropertyNames(globalThis)) {
      if (sandboxBase.has(name)) continue;
      try {
        delete globalThis[name];
        removed++;
      } catch (_) {
        /* 不可配置的宿主属性不允许删除 */
      }
    }
    return removed;
  };

  // 场景属性：剥掉 React 自己管理的东西（children 在树结构里，ref 由 React 持有）。
  const sceneProps = (p) => {
    if (!p) return {};
    const o = {};
    for (const k of Object.keys(p)) {
      if (k === 'children' || k === 'ref' || k === 'key') continue;
      o[k] = p[k];
    }
    return o;
  };

  // ---- 属性值净化 ----------------------------------------------------------
  // props 里可以出现 React 元素（`through={plate}`、`face(plate, '+y')` 里的 of），
  // 旧路径下它们是自建的 {__el,…}，dump 成 {t,p,c}。这里必须转成同形的树，否则
  // 构建器（to_el / face_of）拿不到几何。组件元素无法在属性位置求值 → 丢弃。
  // React 19 把元素标记改成了 Symbol.for('react.transitional.element')（19 之前是
  // 'react.element'）。两个都认，并留一条描述规则以防再改名。
  const isElement = (v) => {
    if (v === null || typeof v !== 'object') return false;
    const s = v.$$typeof;
    if (typeof s !== 'symbol') return false;
    if (s === Symbol.for('react.element') || s === Symbol.for('react.transitional.element')) {
      return true;
    }
    const d = String(s); // "Symbol(react.…element)"
    return d.indexOf('react.') === 7 && d.indexOf('element') > 0;
  };

  function childrenTree(children) {
    const out = [];
    const push = (c) => {
      if (c === null || c === undefined || typeof c === 'boolean') return;
      if (Array.isArray(c)) {
        c.forEach(push);
        return;
      }
      const t = elementTree(c);
      if (t !== undefined && t !== null) out.push(t);
    };
    push(children);
    return out;
  }

  function elementTree(el) {
    if (Array.isArray(el)) return childrenTree(el);
    if (!isElement(el)) return sanitize(el);
    // Fragment 在属性位置（face(<><sphere/></>, '+z')）也是透明容器：与渲染路径
    // 的 fragment 分支同形。此前返回 undefined → JSON 丢键 → Rust 侧缺参报错。
    if (el.type === React.Fragment || el.type === Symbol.for('react.fragment')) {
      return { t: 'fragment', p: {}, c: childrenTree(el.props.children) };
    }
    if (typeof el.type !== 'string') return undefined;
    return { t: el.type, p: sanitize(sceneProps(el.props)), c: childrenTree(el.props.children) };
  }

  // 快照净化：只留 JSON 安全、场景可理解的值。函数 → undefined（丢弃）。
  function sanitize(v, depth) {
    depth = depth || 0;
    if (v === null) return null;
    if (v === undefined) return undefined;
    const t = typeof v;
    if (t === 'number' || t === 'string' || t === 'boolean') return v;
    if (t === 'function') return undefined;
    if (Array.isArray(v))
      return v.map((x) => {
        const s = sanitize(x, depth + 1);
        return s === undefined ? null : s;
      });
    if (t === 'object') {
      if (isElement(v)) return elementTree(v);
      if (depth > 16) return undefined;
      const o = {};
      for (const k of Object.keys(v)) {
        const s = sanitize(v[k], depth + 1);
        if (s !== undefined) o[k] = s;
      }
      return o;
    }
    return undefined;
  }

  function makeHost(counters, nextId) {
    let updatePriority = R.lanes.DefaultEventPriority;
    // E1：事件优先级——dispatch（click 等离散事件）期间置 Discrete。
    let eventPriority = R.lanes.DefaultEventPriority;
    // 可观测性：resolveUpdatePriority 的取值分布（测试断言离散事件走离散车道）。
    const laneStats = { discrete: 0, continuous: 0, default: 0, other: 0 };
    // 实例版本：每次创建 / props 更新 / 结构变更（增删移动子节点）都在受影响实例上
    // 递增。快照时自底向上取子树最大值 __v —— 子树任何变化都会沿祖先链抬高 __v，
    // 宿主据此复用未变子树的构建产物（增量构建，见 docs/jsx-css-host.md）。
    let vc = 0;
    const bump = (n) => {
      n.v = ++vc;
    };

    const mkHost = (type, props) => {
      counters.create++;
      return { kind: 'host', id: nextId(), type, props: sceneProps(props), children: [], v: ++vc };
    };
    const mkText = (text) => {
      counters.text++;
      return { kind: 'text', id: nextId(), text: String(text), children: [], v: ++vc };
    };
    const detach = (p, c) => {
      const i = p.children.indexOf(c);
      if (i >= 0) p.children.splice(i, 1);
    };
    const insertBefore = (p, c, before) => {
      const i = p.children.indexOf(before);
      p.children.splice(i < 0 ? p.children.length : i, 0, c);
    };

    return {
      // --- 标识与环境 ---
      rendererVersion: R.version,
      rendererPackageName: 'cga-react',
      extraDevToolsConfig: null,
      isPrimaryRenderer: false,
      warnsIfNotActing: false,
      noTimeout: -1,
      supportsMutation: true,
      supportsPersistence: false,
      supportsHydration: false,
      supportsMicrotasks: true,
      scheduleMicrotask: (fn) => globalThis.queueMicrotask(fn),
      supportsTestSelectors: false,

      // --- 调度与优先级 ---
      scheduleTimeout: (fn, d) => globalThis.setTimeout(fn, d),
      cancelTimeout: (id) => globalThis.clearTimeout(id),
      getCurrentEventPriority: () => eventPriority,
      resolveUpdatePriority: () => {
        const p = updatePriority;
        if (p === R.lanes.DiscreteEventPriority) laneStats.discrete++;
        else if (p === R.lanes.ContinuousEventPriority) laneStats.continuous++;
        else if (p === R.lanes.DefaultEventPriority) laneStats.default++;
        else laneStats.other++;
        return p;
      },
      setCurrentUpdatePriority: (p) => {
        updatePriority = p;
      },
      // 非标准键（reconciler 不读）：laneStats 引用与事件优先级写入器，
      // 供同文件的 session.dispatch/lanes 访问（词法作用域在 makeHost 内）。
      __laneStats: laneStats,
      __setEventPriority: (p) => {
        eventPriority = p;
      },
      getCurrentUpdatePriority: () => updatePriority,
      trackSchedulerEvent: () => {},
      resolveEventType: () => null,
      resolveEventTimeStamp: () => -1.1,
      shouldAttemptEagerTransition: () => false,

      // --- 上下文 ---
      getRootHostContext: () => ({}),
      getChildHostContext: () => ({}),
      getPublicInstance: (i) => i,
      getInstanceFromNode: () => null,
      preparePortalMount: () => {},

      // --- 创建 / 更新 / 提交 ---
      createInstance: (type, props) => mkHost(type, props),
      createTextInstance: (text) => mkText(text),
      appendInitialChild: (p, c) => {
        bump(p);
        p.children.push(c);
      },
      finalizeInitialChildren: () => false,
      shouldSetTextContent: () => false,
      prepareForCommit: () => null,
      resetAfterCommit: () => {},
      commitUpdate: (inst, type, prevProps, nextProps) => {
        counters.update++;
        inst.props = sceneProps(nextProps);
        bump(inst);
      },
      commitTextUpdate: (t, o, n) => {
        t.text = String(n);
        bump(t);
      },
      commitMount: () => {
        counters.mount++;
      },
      resetTextContent: () => {},
      appendChild: (p, c) => {
        bump(p);
        p.children.push(c);
      },
      appendChildToContainer: (p, c) => {
        bump(p);
        p.children.push(c);
      },
      insertBefore: (p, c, b) => {
        counters.insert++;
        bump(p);
        insertBefore(p, c, b);
      },
      insertInContainerBefore: (p, c, b) => {
        counters.insert++;
        bump(p);
        insertBefore(p, c, b);
      },
      removeChild: (p, c) => {
        counters.remove++;
        bump(p);
        detach(p, c);
      },
      removeChildFromContainer: (p, c) => {
        counters.remove++;
        bump(p);
        detach(p, c);
      },
      clearContainer: (c) => {
        bump(c);
        c.children.length = 0;
      },
      clearSuspenseBoundary: (p, s) => {
        bump(p);
        detach(p, s);
      },
      hideInstance: (i) => {
        i.hidden = true;
      },
      unhideInstance: (i) => {
        i.hidden = false;
      },
      hideTextInstance: (i) => {
        i.hidden = true;
      },
      unhideTextInstance: (i) => {
        i.hidden = false;
      },
      detachDeletedInstance: () => {},

      // --- 异步动作 / 可挂起提交（本渲染器不挂起） ---
      maySuspendCommit: () => false,
      maySuspendCommitOnUpdate: () => false,
      maySuspendCommitInSyncRender: () => false,
      preloadInstance: () => true,
      startSuspendingCommit: () => {},
      suspendInstance: () => {},
      suspendOnActiveViewTransition: () => false,
      waitForCommitToBeReady: () => null,
      getSuspendedCommitReason: () => null,
      resetFormInstance: () => {},
      bindToConsole: (method, args) => {
        const fn = console[method] || console.log;
        fn.apply(console, args);
      },

      // --- 过渡态 ---
      NotPendingTransition: null,
      HostTransitionContext: React.createContext(null),

      // --- test selector 面（dev 专用；只是补齐接口） ---
      findFiberRoot: () => null,
      getBoundingRect: () => ({ x: 0, y: 0, width: 0, height: 0 }),
      getTextContent: () => '',
      isHiddenSubtree: () => false,
      matchAccessibilityRole: () => false,
      setFocusIfFocusable: () => false,
      setupIntersectionObserver: () => ({ disconnect() {} }),

      // --- View Transition 面（实验特性未开；补齐接口） ---
      applyViewTransitionName: () => {},
      restoreViewTransitionName: () => {},
      cancelViewTransitionName: () => {},
      cancelRootViewTransitionName: () => {},
      restoreRootViewTransitionName: () => {},
      measureInstance: () => ({ x: 0, y: 0, width: 0, height: 0 }),
      measureClonedInstance: () => ({ x: 0, y: 0, width: 0, height: 0 }),
      wasInstanceInViewport: () => false,
      hasInstanceChanged: () => false,
      hasInstanceAffectedParent: () => false,
      startViewTransition: (cb) => {
        cb();
      },
    };
  }

  function createSession(opts) {
    opts = opts || {};
    const errors = [];
    const counters = { create: 0, text: 0, update: 0, remove: 0, insert: 0, mount: 0 };
    let idc = 0;
    const nextId = () => ++idc;
    const container = { kind: 'root', id: 0, type: '#root', props: {}, children: [] };
    const hostConfig = makeHost(counters, nextId);
    const reconciler = R.Reconciler(hostConfig);
    // E3：显式 injectIntoDevTools（渲染器侧义务，ReactDOM 同款）——reconciler
    // 不会自动注册。之后每次提交 reconciler 会回调 hook.onCommitFiberRoot。
    if (typeof reconciler.injectIntoDevTools === 'function') reconciler.injectIntoDevTools();
    const tag = opts.tag === 'legacy' ? R.roots.LegacyRoot : R.roots.ConcurrentRoot;

    // 宿主输入通道：用 React context，而不是可变闭包。改值只让 useContext(HostInput)
    // 的组件重渲染，其余子树照常 bailout —— 这正是"props 变化 → 局部重渲染"。
    const InputCtx = React.createContext({});
    globalThis.HostInput = InputCtx;
    let input = {};

    // 根包装组件与当前根元素。Root 的身份在会话内固定，于是每次 update 都重渲染
    // 同一个组件类型，React 只 diff 它返回的元素树（组件身份不丢，hook 状态得以保留）。
    let rootElement = null;
    let sandboxOn = false;
    const Root = function __CgaRoot() {
      return React.createElement(InputCtx.Provider, { value: input }, rootElement);
    };
    const root = reconciler.createContainer(
      container,
      tag,
      null,
      false,
      null,
      '',
      (e) => errors.push('uncaught: ' + (e && (e.stack || e.message) ? e.stack || e.message : String(e))),
      (e) => errors.push('caught: ' + String(e)),
      (e) => errors.push('recoverable: ' + String(e)),
      null,
    );
    const renderRoot = () => reconciler.updateContainer(React.createElement(Root), root, null, null);

    const session = {
      version: R.version,
      rootTag: tag,
      schema: 3,
      // 根元素由 begin() 装填。用一层稳定的 Root 组件包起来：每次 update 都渲染同一个
      // 组件类型，React 只重渲染它返回的元素树（组件身份不丢，hook 状态得以保留）。
      moduleFactory: null,

      begin(moduleSrc, pose, sandbox) {
        const P = pose || {};
        sandboxOn = !!sandbox;
        if (sandboxOn) sandboxBegin();
        session.moduleFactory = new Function('P', moduleSrc + '\n;return __scene;');
        rootElement = session.moduleFactory(P);
        renderRoot();
        if (sandboxOn) sandboxSweep();
        return true;
      },
      update() {
        if (rootElement === null) throw new Error('react-host: update() before begin()');
        renderRoot();
        return true;
      },
      // 宿主推入输入（props 监听）：设值后调用 update()，只有 context 消费者重渲染。
      setInput(inputJson) {
        input = inputJson ? JSON.parse(inputJson) : {};
        return true;
      },
      // 事件派发：自定义渲染器不会自动派发事件，命中谁、派发什么由宿主决定。
      // 从命中实例沿祖先链找第一个 `prop` 处理器并调用（React 的冒泡语义，简化版）。
      dispatch(id, prop, payloadJson) {
        const payload = payloadJson ? JSON.parse(payloadJson) : null;
        const path = [];
        const find = (n) => {
          path.push(n);
          if (n.id === id) return true;
          for (const ch of n.children || []) {
            if (find(ch)) return true;
          }
          path.pop();
          return false;
        };
        let hit = false;
        for (const ch of container.children) {
          if (find(ch)) {
            hit = true;
            break;
          }
        }
        if (!hit) return JSON.stringify({ found: false, reason: 'no such instance ' + id });
        for (let i = path.length - 1; i >= 0; i--) {
          const n = path[i];
          const handler = n.props ? n.props[prop] : undefined;
          if (typeof handler === 'function') {
            // E1：离散事件（click 等）的更新走 DiscreteEventPriority 车道。
            const prevUp = hostConfig.getCurrentUpdatePriority();
            hostConfig.setCurrentUpdatePriority(R.lanes.DiscreteEventPriority);
            hostConfig.__setEventPriority(R.lanes.DiscreteEventPriority);
            try {
              handler({
                type: prop,
                target: { id: n.id, type: n.type },
                currentTarget: { id: n.id, type: n.type },
                payload,
              });
            } finally {
              hostConfig.setCurrentUpdatePriority(prevUp);
              hostConfig.__setEventPriority(R.lanes.DefaultEventPriority);
            }
            return JSON.stringify({ found: true, id: n.id, type: n.type });
          }
        }
        return JSON.stringify({ found: false, reason: 'no ' + prop + ' handler in ancestry of ' + id });
      },
      // 已提交实例清单（宿主拾取/测试用）：id、type、函数型 prop 名。
      instances() {
        const out = [];
        const walk = (n) => {
          const handlers = n.props
            ? Object.keys(n.props).filter((k) => typeof n.props[k] === 'function')
            : [];
          if (n.kind === 'host') out.push({ id: n.id, type: String(n.type), handlers });
          for (const ch of n.children || []) walk(ch);
        };
        for (const ch of container.children) walk(ch);
        return JSON.stringify(out);
      },
      unmount() {
        reconciler.updateContainer(null, root, null, null);
        rootElement = null;
        return true;
      },
      flushSync(fn) {
        reconciler.flushSync(fn);
        return true;
      },
      drain(maxRounds) {
        const r = S.drain(maxRounds);
        if (sandboxOn) sandboxSweep();
        return r;
      },

      // 快照：与旧元素树同形 {t,p,c}；文本节点跳过（当前场景用 props 表达文字）。
      // 每个节点带两个版本：__v = 自身提交版本（创建/props 更新/结构变更时递增），
      // __s = 子树版本（自身与全部后代的 __v 最大值）。宿主据此做增量构建：
      // __s 不变 ⇒ 子树内容不变；祖先链 __v 全不变 ⇒ 该节点的上下文（变换/材质/兄弟
      // 位置）不变。两者同时成立才能整棵复用（见 docs/jsx-css-host.md）。
      // 单个根 → 直接是该元素；多个根 → 包一层 fragment（与旧路径的 fragment 语义一致）；
      // 空场景 → null（交由宿主报"不是元素"，与旧路径一致）。
      snapshot() {
        const clean = (n) => {
          if (n.kind === 'text') return null;
          const out = { t: String(n.type), p: sanitize(n.props), c: [] };
          if (n.kind === 'host') out.__id = n.id; // 实例 id：拾取 → 事件派发的映射
          const own = n.v || 0;
          let sv = own;
          for (const ch of n.children || []) {
            const c = clean(ch);
            if (c !== null) {
              out.c.push(c);
              if (c.__s > sv) sv = c.__s;
            }
          }
          out.__v = own;
          out.__s = sv;
          return out;
        };
        const out = [];
        for (const ch of container.children) {
          const c = clean(ch);
          if (c !== null) out.push(c);
        }
        if (out.length === 0) return 'null';
        if (out.length === 1) return JSON.stringify(out[0]);
        let sv = 0;
        for (const c of out) if (c.__s > sv) sv = c.__s;
        return JSON.stringify({ t: 'fragment', p: {}, c: out, __v: 0, __s: sv });
      },

      // 帧事务：一次做完"动作 + 重渲染 + drain + 快照"，只跨线程一次。
      // action: 'mount'（首帧，带 src/pose/sandbox）/ 'input'（带 input JSON）/
      //         'event'（带 id/prop/payload JSON）/ 'none'（只 drain 当前状态）。
      frame(a) {
        a = a || {};
        let dispatchOut = null;
        if (a.action === 'mount') {
          session.begin(a.src, a.pose, a.sandbox);
        } else if (a.action === 'input') {
          session.setInput(a.input);
          session.update();
        } else if (a.action === 'event') {
          dispatchOut = JSON.parse(session.dispatch(a.id, a.prop, a.payload));
        }
        const stats = session.drain();
        return {
          snapshot: JSON.parse(session.snapshot()),
          stats,
          dispatch: dispatchOut,
          errors: JSON.parse(session.errors()),
        };
      },

      ids() {
        const walk = (n, acc) => {
          acc.push(n.id);
          for (const ch of n.children || []) walk(ch, acc);
          return acc;
        };
        const ids = [];
        for (const ch of container.children) walk(ch, ids);
        return JSON.stringify(ids);
      },
      counters() {
        return JSON.stringify(counters);
      },
      // E1：resolveUpdatePriority 的取值分布（离散事件应走离散车道）。
      lanes() {
        return JSON.stringify(hostConfig.__laneStats);
      },
      // E2：动态 import() 的模块命名空间（bundler 把 import("./x.jsx") 改写成
      // __cga_dyn_import(id)）。
      // E3：DevTools 握手状态（渲染器注册 + 提交计数）。
      devtools() {
        const hook = globalThis.__REACT_DEVTOOLS_GLOBAL_HOOK__;
        const rends = hook ? Array.from(hook.renderers.values()) : [];
        return JSON.stringify({
          renderers: rends.length,
          packageNames: rends.map((r) => r.rendererPackageName),
          commits: hook ? hook.commits.length : 0,
        });
      },
      resetCounters() {
        for (const k of Object.keys(counters)) counters[k] = 0;
        return true;
      },
      errors() {
        return JSON.stringify(errors);
      },
      clearErrors() {
        const n = errors.length;
        errors.length = 0;
        return n;
      },
      logs() {
        return S.formatLogs();
      },
      clearLogs() {
        return S.clearLogs();
      },
    };

    // 方法表：Rust 只认 `__sess.call(payloadJson)` 一个入口，payload = {m, a}。
    // 复用了返回 JSON *字符串* 的会话方法时，这里 parse 成结构化值——信封本身就是
    // JSON，值应当是对象/数组，而不是再套一层字符串。
    const HANDLERS = {
      schema: () => session.schema,
      version: () => session.version,
      begin: (a) => session.begin(a.src, a.pose, a.sandbox),
      update: () => session.update(),
      setInput: (a) => session.setInput(a.json),
      dispatch: (a) => JSON.parse(session.dispatch(a.id, a.prop, a.payload)),
      drain: (a) => session.drain(a && a.maxRounds != null ? a.maxRounds : undefined),
      snapshot: () => JSON.parse(session.snapshot()),
      ids: () => JSON.parse(session.ids()),
      counters: () => JSON.parse(session.counters()),
      resetCounters: () => session.resetCounters(),
      errors: () => JSON.parse(session.errors()),
      clearErrors: () => session.clearErrors(),
      logs: () => JSON.parse(session.logs()),
      instances: () => JSON.parse(session.instances()),
      lanes: () => JSON.parse(session.lanes()),
      devtools: () => JSON.parse(session.devtools()),
      unmount: () => session.unmount(),
      frame: (a) => session.frame(a),
    };
    session.call = (payloadJson) =>
      envelope(() => {
        const req = payloadJson ? JSON.parse(payloadJson) : {};
        const handler = HANDLERS[req.m];
        if (!handler) {
          const e = new Error('unknown host call: ' + req.m);
          e.cgaKind = 'internal';
          throw e;
        }
        return handler(req.a || {});
      });
    return session;
  }

  globalThis.CGA_REACT_HOST = {
    version: R.version,
    schema: 3,
    createSession,
    // 供宿主在派发事件时临时加高优先级（P2 事件驱动用）
    lanes: R.lanes,
  };

  // 宿主唯一的求值入口（原始表达式 / 测试驱动用）：异常（含语法错误）转成结构化信封。
  // 注意：boa 里 Error 的 stack 不含 message，errorInfo 会自动拼上。
  globalThis.__cga_run = (fn) => envelope(fn);
})();
