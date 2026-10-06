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
(function () {
  'use strict';

  const R = globalThis.CGA_REACT;
  const S = globalThis.CGA_SCHED;
  if (!R || !S) {
    throw new Error('react-host: load react-shim.js and react-runtime.*.js first');
  }
  const React = R.React;

  // 场景源码可以像旧 PRELUDE 一样直接用 h()/Fragment（P1 的 PRELUDE 会覆盖为其增强版）。
  globalThis.React = React;
  globalThis.h = React.createElement;
  globalThis.Fragment = React.Fragment;

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

  function makeHost(counters, nextId) {
    let updatePriority = R.lanes.DefaultEventPriority;

    const mkHost = (type, props) => {
      counters.create++;
      return { kind: 'host', id: nextId(), type, props: sceneProps(props), children: [] };
    };
    const mkText = (text) => {
      counters.text++;
      return { kind: 'text', id: nextId(), text: String(text), children: [] };
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
      getCurrentEventPriority: () => R.lanes.DefaultEventPriority,
      resolveUpdatePriority: () => R.lanes.DefaultEventPriority,
      setCurrentUpdatePriority: (p) => {
        updatePriority = p;
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
        p.children.push(c);
      },
      finalizeInitialChildren: () => false,
      shouldSetTextContent: () => false,
      prepareForCommit: () => null,
      resetAfterCommit: () => {},
      commitUpdate: (inst, type, prevProps, nextProps) => {
        counters.update++;
        inst.props = sceneProps(nextProps);
      },
      commitTextUpdate: (t, o, n) => {
        t.text = String(n);
      },
      commitMount: () => {
        counters.mount++;
      },
      resetTextContent: () => {},
      appendChild: (p, c) => {
        p.children.push(c);
      },
      appendChildToContainer: (p, c) => {
        p.children.push(c);
      },
      insertBefore: (p, c, b) => {
        counters.insert++;
        insertBefore(p, c, b);
      },
      insertInContainerBefore: (p, c, b) => {
        counters.insert++;
        insertBefore(p, c, b);
      },
      removeChild: (p, c) => {
        counters.remove++;
        detach(p, c);
      },
      removeChildFromContainer: (p, c) => {
        counters.remove++;
        detach(p, c);
      },
      clearContainer: (c) => {
        c.children.length = 0;
      },
      clearSuspenseBoundary: (p, s) => detach(p, s),
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
    const tag = opts.tag === 'legacy' ? R.roots.LegacyRoot : R.roots.ConcurrentRoot;

    // 根包装组件与当前根元素。Root 的身份在会话内固定，于是每次 update 都重渲染
    // 同一个组件类型，React 只 diff 它返回的元素树（组件身份不丢，hook 状态得以保留）。
    let rootElement = null;
    const Root = function __CgaRoot() {
      return rootElement;
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
      // 根元素由 begin() 装填。用一层稳定的 Root 组件包起来：每次 update 都渲染同一个
      // 组件类型，React 只重渲染它返回的元素树（组件身份不丢，hook 状态得以保留）。
      moduleFactory: null,

      begin(moduleSrc, pose) {
        const P = pose || {};
        session.moduleFactory = new Function('P', moduleSrc + '\n;return __scene;');
        rootElement = session.moduleFactory(P);
        renderRoot();
        return true;
      },
      update() {
        if (rootElement === null) throw new Error('react-host: update() before begin()');
        renderRoot();
        return true;
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
        return S.drain(maxRounds);
      },

      // 快照：与旧元素树同形 {t,p,c}；文本节点跳过（当前场景用 props 表达文字）。
      snapshot() {
        const clean = (n) => {
          if (n.kind === 'text') return null;
          const out = { t: String(n.type), p: sanitize(n.props), c: [] };
          for (const ch of n.children || []) {
            const c = clean(ch);
            if (c !== null) out.c.push(c);
          }
          return out;
        };
        const out = [];
        for (const ch of container.children) {
          const c = clean(ch);
          if (c !== null) out.push(c);
        }
        return JSON.stringify(out);
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
    return session;
  }

  // 快照净化：只留 JSON 安全、场景可理解的值。函数 / React 元素 → undefined（丢弃）。
  function sanitize(v, depth) {
    depth = depth || 0;
    if (v === null) return null;
    if (v === undefined) return undefined;
    const t = typeof v;
    if (t === 'number' || t === 'string' || t === 'boolean') return v;
    if (t === 'function') return undefined;
    if (Array.isArray(v)) return v.map((x) => {
      const s = sanitize(x, depth + 1);
      return s === undefined ? null : s;
    });
    if (t === 'object') {
      if (v.$$typeof) return undefined; // React 元素不可作为场景属性
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

  globalThis.CGA_REACT_HOST = {
    version: R.version,
    createSession,
    // 供宿主在派发事件时临时加高优先级（P2 事件驱动用）
    lanes: R.lanes,
  };

  // 宿主唯一的求值入口：异常（含语法错误）转成 '__CGA_ERR__…' 文本返回，
  // Rust 侧只处理字符串，不接触引擎级异常对象。
  globalThis.__cga_run = (fn) => {
    try {
      return '__CGA_OK__' + String(fn());
    } catch (e) {
      return '__CGA_ERR__' + String((e && (e.stack || e.message)) || e);
    }
  };
})();
