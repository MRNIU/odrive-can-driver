// Copyright The odrive-can-protocol Contributors

//! Linux SocketCAN 4 原生帧转换，适用于同步与 Tokio 接口。
//!
//! `(&encoded).into()` 生成 Classic `CanFrame`。`FrameRef::try_from` 直接接收
//! `CanFrame` 或 `CanAnyFrame`，保留 FD 标志，错误通知返回 `ErrorFrame`，不执行 I/O。

use embedded_can::{Frame, StandardId};
use socketcan::{CanAnyFrame, CanFrame};

use crate::protocol::{EncodedFrame, FramePayload, FrameRef};

impl From<&EncodedFrame> for CanFrame {
    fn from(frame: &EncodedFrame) -> Self {
        // 编码器保证标准 ID、DLC <= 8；原生构造器接受这些输入。
        let id = StandardId::new(frame.id()).expect("encoded standard CAN ID");
        if frame.is_remote() {
            Self::new_remote(id, usize::from(frame.dlc()))
        } else {
            Self::new(id, frame.data())
        }
        .expect("valid encoded Classic CAN frame")
    }
}

/// SocketCAN 输入不能表示为协议帧的原因。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FromSocketcanError {
    /// Linux 总线错误通知；应用应独立处理，不是 Heartbeat 中的设备错误位。
    ErrorFrame,
}

impl core::fmt::Display for FromSocketcanError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("SocketCAN bus error notification is not a protocol frame")
    }
}

impl core::error::Error for FromSocketcanError {}

impl<'a> TryFrom<&'a CanFrame> for FrameRef<'a> {
    type Error = FromSocketcanError;

    fn try_from(frame: &'a CanFrame) -> Result<Self, Self::Error> {
        let payload = match frame {
            CanFrame::Data(frame) => FramePayload::Data(frame.data()),
            CanFrame::Remote(frame) => FramePayload::Remote {
                dlc: frame.dlc() as u8,
            },
            CanFrame::Error(_) => return Err(FromSocketcanError::ErrorFrame),
        };
        Ok(Self {
            id: frame.id().into(),
            payload,
        })
    }
}

impl<'a> TryFrom<&'a CanAnyFrame> for FrameRef<'a> {
    type Error = FromSocketcanError;

    fn try_from(frame: &'a CanAnyFrame) -> Result<Self, Self::Error> {
        let payload = match frame {
            CanAnyFrame::Normal(frame) => FramePayload::Data(frame.data()),
            CanAnyFrame::Remote(frame) => FramePayload::Remote {
                dlc: frame.dlc() as u8,
            },
            CanAnyFrame::Fd(frame) => FramePayload::Fd(frame.data()),
            CanAnyFrame::Error(_) => return Err(FromSocketcanError::ErrorFrame),
        };
        Ok(Self {
            id: frame.id().into(),
            payload,
        })
    }
}
