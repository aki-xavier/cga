//! JSX + CSS scene host (docs/jsx-css-host.md: real JS via
//! boa, JSX compiled by swc, CSS parsed by lightningcss).
//!
//! Pipeline: .jsx source → swc (parse JSX → h() calls) → boa executes real
//! JS (components, map, Math, …) → element tree → scene builder (reuses the
//! geometry/material builders and the kinematics registry) → `SceneRun`.
//!
//! Conventions:
//! - The scene is the `export default <element>` value.
//! - Builtin elements are PascalCase globals bound to lowercase strings, or
//!   lowercase tags directly: `<Sphere r={1}/>` ≡ `<sphere r={1}/>`.
//! - CSS matches by tag / `.class` / `#id`; material properties inherit down
//!   the element tree; inline props beat matched CSS.
//!
//! Layering (2026-10-09 拆分，见 docs/module-review.md）：本文件只做模块
//! 声明与门面再导出，实现分五层——
//!
//! | 模块 | 职责 |
//! | --- | --- |
//! | [`compile`] | swc：JSX 解析 / `h()` 改写 / 导入打包 / codegen / `PRELUDE` |
//! | [`element`] | 元素值层：boa JSON → `El` 树、prop 读取器、`ArgValue` 转换 |
//! | [`css`] | CSS：选择器编译/匹配/级联/值转换 |
//! | [`builder`] | `Builder`：元素树 → `SceneRun`（几何/材质/运动学/增量复用） |
//! | [`session`] | boa 执行、`run_jsx*` 入口、惰性分析、增量缓存、`SceneSession` |
//!
//! 对外 API（`cga_host::*`）路径不变：都在本模块重导出。

mod builder;
mod compile;
mod css;
mod element;
mod session;

#[cfg(test)]
mod tests;

// 门面再导出：对外 API 与测试所需的全部名字。
pub use self::compile::compile_jsx_with;
pub use self::element::El;
pub use self::session::{
    render_jsx_png, render_jsx_png_mode, run_jsx, run_jsx_pose, run_jsx_pose_sandbox, BuildCache,
    BuildStats, HeadlessImage, SceneSession,
};
