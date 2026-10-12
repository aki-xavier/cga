/// 渲染模式（供宿主使用者直接引用）。
pub use cga_gpu::RenderMode;

pub mod react;
pub use react::{
    DrainStats, ErrorKind, FrameAction, FrameOutcome, Owned, Pooled, ReactError, ReactSession,
    Runtime, Sandbox, SessionOptions, SCENE_SCHEMA,
};

pub mod scene_build;
pub use scene_build::*;

pub mod scene_report;

pub mod collision;
pub use collision::*;

pub mod jsx;
pub use jsx::*;

pub mod jsx_gen;
pub use jsx_gen::*;

pub mod export;
pub use export::*;

pub mod urdf;
pub use urdf::*;

/// 测试共用助手：FNV-1a 64 指纹（渲染金标比对用，`jsx` 与 `collision` 测试共享）。
#[cfg(test)]
pub(crate) mod test_util {
    pub(crate) fn fnv1a(bytes: &[u8]) -> u64 {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for b in bytes {
            h ^= u64::from(*b);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
        h
    }
}
