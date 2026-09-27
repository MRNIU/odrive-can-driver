// Copyright The odrive-can-driver Contributors

//! Linux SocketCAN 4 的非阻塞薄封装。
//!
//! 调用方仍拥有套接字和共享总线的调度策略。此模块不清空接收队列、不配置接口；发送成功
//! 只表示内核已接受帧。明确的 `WouldBlock` 会回到 `Prepared`；其他 `write_frame` I/O 错误
//! 都会让发送 guard 析构为 `Unknown`，因而不能自动重试。

extern crate std;

use std::{fmt, io};

use socketcan::{CanAnyFrame, CanFdSocket, CanFrame, Socket};

use crate::{
    AttemptError, BeginSendError, Driver, IngestResult, OperationId,
    protocol::{FrameRef, compat::socketcan::FromSocketcanError},
};

/// 一个已设为非阻塞模式的 Linux CAN FD 套接字。
///
/// 使用 [`CanFdSocket`] 以便接收 Classic CAN、RTR、CAN FD 和内核错误通知。创建此类型不
/// 配置 Linux 接口、位速率或过滤器；这些都属于应用的总线所有权。
#[derive(Debug)]
pub struct SocketCan {
    socket: CanFdSocket,
}

impl SocketCan {
    /// 打开 `interface`，并将套接字设为非阻塞模式。
    ///
    /// 此调用只打开 SocketCAN 描述符并设置该描述符的阻塞模式；接口必须已由调用方创建、
    /// 启用和配置。
    pub fn open(interface: &str) -> io::Result<Self> {
        Self::from_socket(CanFdSocket::open(interface)?)
    }

    /// 接管一个调用方已配置的 CAN FD 套接字，并将此描述符设为非阻塞模式。
    ///
    /// 此方法不会创建、启用、重配接口，也不会改变套接字的过滤器或错误过滤器。调用方可先
    /// 通过 [`SocketCan::socket`] 设置所需的本 socket 选项，再将原始帧交由 [`SocketCan::receive`]
    /// 分类。
    pub fn from_socket(socket: CanFdSocket) -> io::Result<Self> {
        socket.set_nonblocking(true)?;
        Ok(Self { socket })
    }

    /// 返回底层套接字，供调用方配置此描述符专属的 SocketCAN 选项。
    ///
    /// SocketCAN 默认不接收错误通知。若应用要让 [`SocketCan::receive`] 返回分类为错误通知的
    /// 原生帧，应通过此引用调用 `SocketOptions::set_error_filter`，例如设置 `ERR_MASK_ALL`。
    /// 这只改变该套接字描述符，不重配共享 CAN 接口。
    pub fn socket(&self) -> &CanFdSocket {
        &self.socket
    }

    /// 发送一个已准备的操作。
    ///
    /// `now_ms` 在开始发送前采样一次；SocketCAN 成功接受帧或明确报告 `WouldBlock` 后再采样
    /// 一次。返回 `Ok(())` 时，`Driver` 已在后一次采样时刻记录 `Submitted`，它只表示本地
    /// SocketCAN 已接受帧，不表示 ODrive 已执行。`WouldBlock` 明确表示未入队，操作回到
    /// `Prepared`；其他 I/O 错误不能证明帧没有离开进程，因此发送 guard 析构为 `Unknown`。
    pub fn send<F>(
        &self,
        driver: &mut Driver,
        id: OperationId,
        mut now_ms: F,
    ) -> Result<(), SocketCanSendError>
    where
        F: FnMut() -> u64,
    {
        let attempt = driver
            .begin_send(id, now_ms())
            .map_err(SocketCanSendError::Begin)?;
        let frame: CanFrame = attempt.frame().into();
        match self.socket.write_frame(&frame) {
            Ok(()) => attempt
                .submitted(now_ms())
                .map_err(SocketCanSendError::Attempt),
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                attempt
                    .would_block(now_ms())
                    .map_err(SocketCanSendError::Attempt)?;
                Err(SocketCanSendError::Io(error))
            }
            Err(error) => Err(SocketCanSendError::Io(error)),
        }
    }

    /// 读取一个原生 SocketCAN 帧并交给共享 driver 分类。
    ///
    /// 由于 [`SocketCan::open`] 始终设置非阻塞模式，没有帧时返回 `ErrorKind::WouldBlock`。
    /// `Normal`、`Remote`、`Fd` 的无关帧，以及 `Error` 错误通知，都会以原始
    /// [`CanAnyFrame`] 交还调用方。`timestamp_ms` 只在成功出队后调用，记录调用方的观察时刻；
    /// 套接字接收队列可能已有更早帧，本方法不把该时刻声明为物理线上采样时间，也不清空队列。
    pub fn receive(
        &self,
        driver: &mut Driver,
        timestamp_ms: impl FnOnce() -> u64,
    ) -> io::Result<SocketCanReceive> {
        let frame = self.socket.read_frame()?;
        Ok(receive_frame(driver, frame, timestamp_ms()))
    }
}

/// SocketCAN 发送在开始、I/O 或共享操作状态记录阶段的错误。
#[derive(Debug)]
pub enum SocketCanSendError {
    /// 操作不存在、尚未准备，或发送前的时钟和期限检查失败。
    Begin(BeginSendError),
    /// 记录底层成功或 `WouldBlock` 结果时的时钟或期限错误。
    ///
    /// 通过操作报告区分已接受但结果未知与明确未接受，不能仅凭此变体判定是否发送。
    Attempt(AttemptError),
    /// SocketCAN 写入返回的原始 I/O 错误。
    ///
    /// `ErrorKind::WouldBlock` 已明确未入队，对应操作回到 `Prepared`；其他 I/O 错误对应
    /// 操作保留为 `Unknown`。
    Io(io::Error),
}

impl fmt::Display for SocketCanSendError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Begin(error) => write!(formatter, "cannot begin SocketCAN send: {error:?}"),
            Self::Attempt(error) => write!(formatter, "cannot record SocketCAN send: {error:?}"),
            Self::Io(error) => write!(formatter, "SocketCAN write failed: {error}"),
        }
    }
}

impl std::error::Error for SocketCanSendError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            Self::Begin(_) | Self::Attempt(_) => None,
        }
    }
}

/// 一个被 SocketCAN 后端读取的帧及其分类。
#[derive(Clone, Debug, PartialEq)]
pub struct SocketCanReceive {
    /// 原始 SocketCAN 帧，始终由调用方保留所有权。
    pub frame: CanAnyFrame,
    /// 协议分类；`Ok` 包含成功、解码错误或无关帧，`Err` 表示 Linux 错误通知。
    pub classification: Result<IngestResult, FromSocketcanError>,
}

/// 将一个已由调用方读取的原生帧交给 driver 分类。
///
/// 这个函数不做 I/O，便于共享总线的单一接收循环统一读取并分发。所有 `CanAnyFrame`
/// 变体都会在返回值中原样保留；`Error` 不会成为 ODrive 协议消息。
pub fn receive_frame(
    driver: &mut Driver,
    frame: CanAnyFrame,
    received_at_ms: u64,
) -> SocketCanReceive {
    let classification = FrameRef::try_from(&frame)
        .map(|protocol_frame| driver.ingest(protocol_frame, received_at_ms));
    SocketCanReceive {
        frame,
        classification,
    }
}
