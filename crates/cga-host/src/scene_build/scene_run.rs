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
    /// 与 `scene.objects` 平行：每个对象来自的 React 宿主实例 id
    /// （拾取 → 事件派发的映射；合成/复用的克隆对象继承来源的 id）。
    pub object_instances: Vec<Option<i64>>,
}
