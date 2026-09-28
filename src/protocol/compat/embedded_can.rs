// Copyright The odrive-can-protocol Contributors

//! `embedded-can` 0.4 的 Classic CAN 帧转换，可用于 bxCAN 0.8 等驱动。
//!
//! 调用方须确保输入类型只表示 Classic 数据帧或 RTR。此 trait 不暴露 FDF 或错误帧标志，
//! 因此 Embassy、SocketCAN 混合帧应使用专用的 `TryFrom` 实现，不能仅凭长度判断 CAN FD。

use crate::protocol::{EncodedFrame, FrameId, FramePayload, FrameRef};
use embedded_can::{Frame, Id, StandardId};

impl EncodedFrame {
    /// 转换为实现 `embedded_can::Frame` 的 Classic CAN 帧。
    ///
    /// `F::new` 必须构造 Classic 数据帧，`F::new_remote` 必须构造 RTR，例如 `bxcan::Frame`。
    /// 不适用于会构造 CAN FD 的类型。`None` 表示驱动构造器拒绝此帧；不发送数据。
    pub fn to_embedded_can<F: Frame>(&self) -> Option<F> {
        let id = StandardId::new(self.id())?;
        if self.is_remote() {
            F::new_remote(id, usize::from(self.dlc()))
        } else {
            F::new(id, self.data())
        }
    }
}

/// Classic CAN 帧的 DLC 或有效数据长度不符合驱动 trait 合同。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InvalidFrameLength;

impl core::fmt::Display for InvalidFrameLength {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("invalid Classic CAN frame length")
    }
}

impl core::error::Error for InvalidFrameLength {}

impl<'a> FrameRef<'a> {
    /// 借用已确认是 Classic CAN 数据或 RTR 的驱动帧。
    ///
    /// 调用方必须先从原生类型排除 CAN FD 和总线错误；此 trait 不提供相应标志。
    /// DLC 必须为 `0..=8`，数据缓冲至少含 DLC 字节，否则返回错误。
    /// 扩展 ID 原样保留，RTR 保留 DLC 且不读取数据，数据帧只借用有效前缀。
    pub fn from_classic_embedded_can<F: Frame>(frame: &'a F) -> Result<Self, InvalidFrameLength> {
        let dlc = frame.dlc();
        if dlc > 8 {
            return Err(InvalidFrameLength);
        }
        let payload = if frame.is_remote_frame() {
            FramePayload::Remote { dlc: dlc as u8 }
        } else {
            FramePayload::Data(frame.data().get(..dlc).ok_or(InvalidFrameLength)?)
        };
        Ok(Self {
            id: frame.id().into(),
            payload,
        })
    }
}

impl From<Id> for FrameId {
    fn from(id: Id) -> Self {
        match id {
            Id::Standard(id) => Self::Standard(id.as_raw()),
            Id::Extended(id) => Self::Extended(id.as_raw()),
        }
    }
}
