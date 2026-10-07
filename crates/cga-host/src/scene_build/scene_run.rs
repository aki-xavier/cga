use super::*;
use cga_gpu::scene::PerspectiveCamera;

/// A fully evaluated scene: geometry + camera + tag registry + kinematics.
#[derive(Clone, Debug)]
pub struct SceneRun {
    pub scene: Scene,
    pub camera: PerspectiveCamera,
    pub tags: TagRegistry,
    pub kinematics: Kinematics,
    /// 分组注册表：id → 名字（0 = "" 未分组）。append-only，跨帧 id 稳定，
    /// `Object::group` 是它的下标。
    pub groups: Vec<String>,
}
