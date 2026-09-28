// Copyright The odrive-can-protocol Contributors

//! Embassy STM32 0.6 原生帧转换。编码结果可通过 `(&encoded).into()` 转为 `Frame` 或
//! `FdFrame`；接收帧通过 `FrameRef::try_from(&frame)` 借用。两种容器发送时均为 Classic。

use embassy_stm32::can::frame::{FdFrame, Frame, Header};
use embedded_can::StandardId;

use crate::protocol::{EncodedFrame, FramePayload, FrameRef};

impl From<&EncodedFrame> for Frame {
    fn from(frame: &EncodedFrame) -> Self {
        // 编码器保证标准 ID、DLC <= 8 和有效数据长度。
        Self::new(encoded_header(frame), frame.data()).expect("valid encoded Classic CAN frame")
    }
}

impl From<&EncodedFrame> for FdFrame {
    fn from(frame: &EncodedFrame) -> Self {
        Self::new(encoded_header(frame), frame.data()).expect("valid encoded Classic CAN frame")
    }
}

fn encoded_header(frame: &EncodedFrame) -> Header {
    let id = StandardId::new(frame.id()).expect("encoded standard CAN ID");
    Header::new(id.into(), frame.dlc(), frame.is_remote())
}

/// Embassy 头部声明的长度超出帧缓冲范围。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FromEmbassyError {
    /// 长度无法从底层缓冲中取得；不会因公开构造器产生的非法头部而 panic。
    InvalidPayloadLength,
}

impl core::fmt::Display for FromEmbassyError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("Embassy frame length exceeds its buffer")
    }
}

impl core::error::Error for FromEmbassyError {}

impl<'a> TryFrom<&'a Frame> for FrameRef<'a> {
    type Error = FromEmbassyError;

    fn try_from(frame: &'a Frame) -> Result<Self, Self::Error> {
        from_parts(frame.header(), frame.raw_data())
    }
}

impl<'a> TryFrom<&'a FdFrame> for FrameRef<'a> {
    type Error = FromEmbassyError;

    fn try_from(frame: &'a FdFrame) -> Result<Self, Self::Error> {
        let header = frame.header();
        if !header.fdcan() && header.rtr() {
            return from_parts(header, &[]);
        }
        // FdFrame::data 会直接按头部切片，先校验以免非法头部导致 panic。
        if header.len() > 64 {
            return Err(FromEmbassyError::InvalidPayloadLength);
        }
        from_parts(header, frame.data())
    }
}

fn from_parts<'a>(header: &Header, data: &'a [u8]) -> Result<FrameRef<'a>, FromEmbassyError> {
    // FDF 优先于 RTR；短 FD 也必须让协议解码器识别并拒绝。
    let payload = if header.fdcan() {
        FramePayload::Fd(data)
    } else if header.rtr() {
        FramePayload::Remote { dlc: header.len() }
    } else {
        FramePayload::Data(
            data.get(..usize::from(header.len()))
                .ok_or(FromEmbassyError::InvalidPayloadLength)?,
        )
    };
    Ok(FrameRef {
        id: (*header.id()).into(),
        payload,
    })
}
