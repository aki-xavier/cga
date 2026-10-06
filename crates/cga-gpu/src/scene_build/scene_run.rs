use super::*;
use crate::scene::PerspectiveCamera;

/// A fully evaluated scene: geometry + camera + tag registry + kinematics.
#[derive(Clone, Debug)]
pub struct SceneRun {
    pub scene: Scene,
    pub camera: PerspectiveCamera,
    pub tags: TagRegistry,
    pub kinematics: Kinematics,
}
