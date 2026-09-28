// Copyright The odrive-can-protocol Contributors

//! ODrive CANSimple 协议库入口，提供版本明确的编解码 API 与硬件无关帧类型。
//!
//! 默认仅使用 core；可选 feature 提供驱动帧转换，不执行设备收发或控制。

#[cfg(all(feature = "socketcan", not(target_os = "linux")))]
compile_error!("feature `socketcan` requires a Linux target");

/// 按 feature 启用的驱动帧转换。
#[cfg(feature = "embedded-can")]
pub mod compat;

/// 与硬件无关的 CAN 帧表示。
pub mod frame;
/// ODrive 固件 `fw-v0.5.1` 的 CANSimple 协议。
pub mod fw_v0_5_1;

pub use frame::{EncodedFrame, FrameId, FramePayload, FrameRef};
pub use fw_v0_5_1::{
    AxisState, Command, DecodeError, EncodeError, Message, NodeId, Query, Response, decode, encode,
};
