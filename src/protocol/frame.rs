// Copyright The odrive-can-protocol Contributors

//! 与 CAN 驱动解耦的帧类型，显式保存 ID 形态、有效载荷与 RTR 信息。
//!
//! 输入借用调用方切片，编码产物使用固定大小缓冲；本模块不进行设备收发。

/// CAN 标识符的格式与原始数值。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameId {
    /// 11 位标准 CAN 标识符。
    Standard(u16),
    /// 29 位扩展 CAN 标识符。
    Extended(u32),
}

/// 借用的 CAN 帧有效载荷。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FramePayload<'a> {
    /// 经典 CAN 数据帧的实际有效字节。
    Data(&'a [u8]),
    /// 经典 CAN 远程帧及其 DLC。
    Remote {
        /// CAN 帧的 DLC，调用方应保留控制器报告的原值。
        dlc: u8,
    },
    /// CAN FD 数据帧的实际有效字节。
    Fd(&'a [u8]),
}

/// 借用的、与具体 CAN 驱动无关的输入帧。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameRef<'a> {
    /// CAN 标识符。
    pub id: FrameId,
    /// 帧种类与有效载荷。
    pub payload: FramePayload<'a>,
}

/// 可直接交给经典 CAN 驱动发送的固定大小帧。
///
/// 此类型只能由协议编码器构造，以保证其标识符与 DLC 适合经典标准 CAN。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EncodedFrame {
    id: u16,
    data: [u8; 8],
    len: u8,
    rtr: bool,
}

impl EncodedFrame {
    /// 创建协议模块内部使用的固定大小帧。
    pub(crate) const fn new(id: u16, data: [u8; 8], len: u8, rtr: bool) -> Self {
        Self { id, data, len, rtr }
    }

    /// 返回 11 位标准 CAN 标识符。
    pub const fn id(&self) -> u16 {
        self.id
    }

    /// 返回数据帧的有效字节；远程帧始终返回空切片。
    pub fn data(&self) -> &[u8] {
        if self.rtr {
            &[]
        } else {
            &self.data[..self.len as usize]
        }
    }

    /// 返回帧 DLC。
    pub const fn dlc(&self) -> u8 {
        self.len
    }

    /// 返回此帧是否为经典 CAN 远程帧。
    pub const fn is_remote(&self) -> bool {
        self.rtr
    }

    /// 将固定大小帧借用为通用输入帧视图。
    pub fn as_ref(&self) -> FrameRef<'_> {
        let payload = if self.rtr {
            FramePayload::Remote { dlc: self.len }
        } else {
            FramePayload::Data(self.data())
        };
        FrameRef {
            id: FrameId::Standard(self.id),
            payload,
        }
    }
}
