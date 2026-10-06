// cga-react 平台层（我们自己的源码，不随 vendor 重新打包）。
//
// React / scheduler 在模块初始化时探测宿主全局。这里把它们替换成**确定性**替身：
// 时间不由真实时钟推进，而是由宿主显式调用 CGA_SCHED.drain() 推进，于是
// "挂载 → 提交 → 副作用" 全部可复现，无需事件循环。
//
// 加载顺序（crates/cga-host/src/react/mod.rs 保证）：shim → runtime → host。
(function () {
  'use strict';

  const S = (globalThis.CGA_SCHED = {
    timers: [],
    microtasks: [],
    logs: [],
    clock: 0,
    seq: 0,
  });

  const str = (x) => {
    if (typeof x === 'string') return x;
    if (x instanceof Error) return x.stack || x.message;
    try {
      return String(x);
    } catch (_) {
      return '<unprintable>';
    }
  };
  const fmt = (args) => Array.prototype.map.call(args, str).join(' ');
  const push = (level, args) => S.logs.push({ level, text: fmt(args) });

  // console：作者期诊断（React 的 invalid hook call 等）也走这里，宿主可读回。
  globalThis.console = {
    log: (...a) => push('log', a),
    info: (...a) => push('info', a),
    debug: (...a) => push('debug', a),
    warn: (...a) => push('warn', a),
    error: (...a) => push('error', a),
  };

  // 定时器只入队；时间由 drain 推进（单调、每跑一个回调 +1）。
  globalThis.setTimeout = (fn, ms = 0, ...args) => {
    const id = ++S.seq;
    S.timers.push({ id, at: S.clock + Math.max(0, ms | 0), fn, args });
    return id;
  };
  globalThis.clearTimeout = (id) => {
    const i = S.timers.findIndex((t) => t.id === id);
    if (i >= 0) S.timers.splice(i, 1);
  };
  globalThis.queueMicrotask = (fn) => {
    S.microtasks.push(fn);
  };
  globalThis.performance = { now: () => S.clock };

  // 把微任务与定时器跑到静止。返回 { rounds, errs }。
  // 回调中的异常被记为 error 日志并计数（不打断其余工作）。
  S.drain = function (maxRounds = 2000) {
    let rounds = 0;
    let errs = 0;
    while (rounds < maxRounds) {
      rounds++;
      let did = false;
      while (S.microtasks.length) {
        const jobs = S.microtasks.splice(0, S.microtasks.length);
        for (const j of jobs) {
          try {
            j();
          } catch (e) {
            push('error', ['microtask: ' + str(e)]);
            errs++;
          }
        }
        did = true;
      }
      if (S.timers.length) {
        S.timers.sort((a, b) => a.at - b.at || a.id - b.id);
        const t = S.timers.shift();
        S.clock = Math.max(S.clock + 1, t.at);
        try {
          t.fn(...t.args);
        } catch (e) {
          push('error', ['timer: ' + str(e)]);
          errs++;
        }
        did = true;
      }
      if (!did) break;
    }
    return { rounds: rounds - 1, errs };
  };

  S.formatLogs = () => JSON.stringify(S.logs);
  S.clearLogs = () => {
    const n = S.logs.length;
    S.logs.length = 0;
    return n;
  };
})();
