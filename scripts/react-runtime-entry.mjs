// 打进 cga-react 运行时的 React 入口（由 scripts/vendor-react.mjs 打包）。
// 运行时只暴露"原料"：React 本体、reconciler 工厂、lane 常量。
// 宿主渲染器适配层在 crates/cga-host/assets/react-host.js（不随这里重新打包）。
import * as React from 'react';
import Reconciler from 'react-reconciler';
import {
  DefaultEventPriority,
  DiscreteEventPriority,
  ContinuousEventPriority,
  IdleEventPriority,
  LegacyRoot,
  ConcurrentRoot,
} from 'react-reconciler/constants';

globalThis.CGA_REACT = {
  version: React.version,
  React,
  Reconciler,
  lanes: {
    DefaultEventPriority,
    DiscreteEventPriority,
    ContinuousEventPriority,
    IdleEventPriority,
  },
  roots: { LegacyRoot, ConcurrentRoot },
};
