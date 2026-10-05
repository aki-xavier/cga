use super::*;

#[derive(Clone, Debug)]
pub struct CgsRun {
    pub scene: Scene,
    pub camera: PerspectiveCamera,
    pub tags: TagRegistry,
    pub kinematics: Kinematics,
}
