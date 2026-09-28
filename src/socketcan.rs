// Copyright The odrive-can-driver Contributors

//! Linux SocketCAN 4 的分离、非阻塞收发端点。
//!
//! [`CanTx`] 和 [`CanRx`] 分别拥有调用方提供的描述符；它们不重配 CAN 接口、不清空 RX
//! 队列，也不在端点之间转移帧。应用仍拥有共享总线的读取、分发和发送调度权。
//!
//! `socketcan 4.0.0` 的 [`CanFdSocket::write_frame`] 在非阻塞描述符上是一次同步系统调用，
//! 因而此模块没有跨轮询的 native 写 future：每次 [`TxAttempt::poll`] 都只授权并执行一次
//! 写入。仅该调用直接返回的 `WouldBlock` 被当作已证明未入队；其他 I/O 错误或调用方丢弃
//! [`TxAttempt`] 都保留为未知副作用。

extern crate std;

use std::{fmt, io};

use socketcan::{CanAnyFrame, CanFdSocket, CanFrame, CanTimestamps, Socket};

use crate::{
    AttemptError, Driver, IngestResult, Instant, TxAttempt as CoreTxAttempt, TxCompletion,
    TxOutcome,
    protocol::{FrameRef, compat::socketcan::FromSocketcanError},
};

/// Linux SocketCAN 发送端点。
#[derive(Debug)]
pub struct CanTx {
    socket: CanFdSocket,
}

impl CanTx {
    /// 打开 `interface` 并将此发送描述符设为非阻塞。
    ///
    /// 此调用不创建、启用或配置接口、位速率、过滤器或错误过滤器。
    pub fn open(interface: &str) -> io::Result<Self> {
        Self::from_socket(CanFdSocket::open(interface)?)
    }

    /// 接管调用方已配置的 CAN FD 描述符，并将该描述符设为非阻塞。
    pub fn from_socket(socket: CanFdSocket) -> io::Result<Self> {
        socket.set_nonblocking(true)?;
        Ok(Self { socket })
    }

    /// 返回底层描述符，供调用方配置此描述符专属的 SocketCAN 选项。
    pub fn socket(&self) -> &CanFdSocket {
        &self.socket
    }

    /// 取回底层描述符。
    pub fn into_socket(self) -> CanFdSocket {
        self.socket
    }

    /// 保存一个 core 已授权的单次发送尝试。
    ///
    /// 返回的值不借用 [`Driver`]。应用可以在 [`TxAttempt::poll`] 调用之间接收、分发帧、
    /// 推进期限或执行取消逻辑。每次 `poll` 仍会重新请求 core 授权，过期或已撤销的尝试
    /// 不能借由旧帧继续发送。
    pub const fn attempt<'tx, 's>(&'tx self, attempt: CoreTxAttempt<'s>) -> TxAttempt<'tx, 's> {
        TxAttempt {
            tx: self,
            attempt: Some(attempt),
        }
    }
}

/// Linux SocketCAN 接收端点。
///
/// 读取使用 `recvmsg` 路径并原样返回 [`CanTimestamps`]。应用可在底层 socket 上启用
/// `SocketOptions::set_recv_timestamp` 或 `SocketOptions::set_timestamping`；这些 Linux
/// 时间戳属于 wall-clock 或适配器时钟域，不能替代传给 core 的单调 [`Instant`]。
#[derive(Debug)]
pub struct CanRx {
    socket: CanFdSocket,
}

impl CanRx {
    /// 打开 `interface` 并将此接收描述符设为非阻塞。
    ///
    /// 此调用不创建、启用或配置接口、位速率、过滤器或错误过滤器。
    pub fn open(interface: &str) -> io::Result<Self> {
        Self::from_socket(CanFdSocket::open(interface)?)
    }

    /// 接管调用方已配置的 CAN FD 描述符，并将该描述符设为非阻塞。
    pub fn from_socket(socket: CanFdSocket) -> io::Result<Self> {
        socket.set_nonblocking(true)?;
        Ok(Self { socket })
    }

    /// 返回底层描述符，供调用方配置此描述符专属的 SocketCAN 选项。
    ///
    /// SocketCAN 默认不交付错误通知。若应用要收到 [`CanAnyFrame::Error`]，可通过此引用
    /// 调用 `SocketOptions::set_error_filter`，例如设置 `ERR_MASK_ALL`。
    pub fn socket(&self) -> &CanFdSocket {
        &self.socket
    }

    /// 取回底层描述符。
    pub fn into_socket(self) -> CanFdSocket {
        self.socket
    }

    /// 读取一帧、保留它的全部 SocketCAN 时间戳，并交给 core 分类。
    ///
    /// `received_at` 是调用方映射到 core 单调微秒时钟域的事件时间。它与下方原生
    /// `timestamps` 分开：后者可能是系统 wall-clock 或硬件时钟，不能隐式混进期限和
    /// 新鲜度判断。没有帧时返回原始 `ErrorKind::WouldBlock`。
    pub fn receive(
        &self,
        driver: &mut Driver<'_>,
        received_at: impl FnOnce(&CanTimestamps) -> Instant,
    ) -> io::Result<CanRxFrame> {
        let (frame, timestamps) = self.socket.read_frame_with_timestamps()?;
        let received_at = received_at(&timestamps);
        Ok(receive_frame(driver, frame, timestamps, received_at))
    }
}

/// 尚未完成的 SocketCAN 发送调用。
///
/// 该对象线性持有 core 尝试。它只会在 [`TxAttempt::poll`] 得到真实 I/O 结果后消费该尝试；
/// 若该对象被丢弃，core 不会把它伪造为取消或未发送。应用应在取消前保存
/// [`TxAttempt::id`]，结束全部原始 I/O 后调用 [`Driver::acknowledge_unknown`]，再提取报告。
#[must_use]
pub struct TxAttempt<'tx, 's> {
    tx: &'tx CanTx,
    attempt: Option<CoreTxAttempt<'s>>,
}

impl<'tx, 's> TxAttempt<'tx, 's> {
    /// 返回所属操作，供调用方在取消前保存身份。
    pub fn id(&self) -> crate::OperationId<'s> {
        match &self.attempt {
            Some(attempt) => attempt.id(),
            None => panic!("SocketCAN TX attempt was already consumed"),
        }
    }

    /// 用一次非阻塞 `write_frame` 推进尝试。
    ///
    /// 此方法短暂借用 driver；成功或 `WouldBlock` 后核心尝试被消费并返回完成结果。非
    /// `WouldBlock` I/O 错误无法证明帧未离开进程，故会先调用 `abandon_attempt` 并使操作
    /// 保持未知。若 core 在 I/O 前拒绝授权，尝试仍由此对象保存，调用方可推进期限或在
    /// 确认没有 native 调用在运行后用 [`TxAttempt::cancel_unsubmitted`] 取消。
    ///
    /// `now` 必须从同一单调微秒时钟采样。此方法在授权前、native 调用返回后记录事件发生
    /// 时刻、以及结果回填时分别采样；不得传入预先采样的常量来掩盖跨越期限的同步调用。
    pub fn poll(
        &mut self,
        driver: &mut Driver<'s>,
        mut now: impl FnMut() -> Instant,
    ) -> Result<TxCompletion<'s>, SocketCanTxError> {
        let attempt = self.attempt.as_ref().ok_or(SocketCanTxError::Consumed)?;
        let authorized_at = now();
        let frame = driver
            .authorize_tx(attempt, authorized_at)
            .map_err(SocketCanTxError::Authorize)?;
        let frame: CanFrame = (&frame).into();

        let outcome = match self.tx.socket.write_frame(&frame) {
            Ok(()) => TxOutcome::Submitted { occurred_at: now() },
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                TxOutcome::WouldBlock { occurred_at: now() }
            }
            Err(error) => {
                let attempt = self.attempt.take().expect("attempt checked above");
                return match driver.abandon_attempt(attempt, now()) {
                    Ok(()) => Err(SocketCanTxError::Io(error)),
                    Err(core) => Err(SocketCanTxError::Abandon { io: error, core }),
                };
            }
        };
        let attempt = self.attempt.take().expect("attempt checked above");
        driver
            .finish_tx(attempt, outcome, now())
            .map_err(SocketCanTxError::Finish)
    }

    /// 记录一个已结束、从未调用 native write 的取消。
    ///
    /// SocketCAN 此端点没有后台 future；因此调用方只可在尚未调用 [`TxAttempt::poll`]，或
    /// `poll` 在实际 write 前被 core 拒绝后调用它。不得以 future 丢弃、超时或普通 I/O
    /// 错误替代此证明。
    pub fn cancel_unsubmitted(
        mut self,
        driver: &mut Driver<'s>,
        occurred_at: Instant,
        processed_at: Instant,
    ) -> Result<(), SocketCanTxError> {
        let attempt = self.attempt.take().ok_or(SocketCanTxError::Consumed)?;
        driver
            .cancel_unsubmitted(attempt, occurred_at, processed_at)
            .map_err(SocketCanTxError::Cancel)
    }

    /// 取出 core 尝试，以便应用在已知外部 native I/O 生命周期下自行处理。
    pub fn into_core(mut self) -> Option<CoreTxAttempt<'s>> {
        self.attempt.take()
    }

    /// 在没有取消且未提交证据时记录未知副作用。
    ///
    /// 先用 [`TxAttempt::id`] 保存身份；本方法结束后，应用还必须确认所有旧 native I/O 已经
    /// 停止，才能调用 [`Driver::acknowledge_unknown`] 解除槽位隔离。它不允许自动重试。
    pub fn abandon(
        mut self,
        driver: &mut Driver<'s>,
        processed_at: Instant,
    ) -> Result<crate::OperationId<'s>, SocketCanTxError> {
        let attempt = self.attempt.take().ok_or(SocketCanTxError::Consumed)?;
        let id = attempt.id();
        driver
            .abandon_attempt(attempt, processed_at)
            .map_err(SocketCanTxError::AbandonCore)?;
        Ok(id)
    }
}

/// SocketCAN 发送路径的错误。
#[derive(Debug)]
pub enum SocketCanTxError {
    /// 已消费该尝试，不能再次轮询或取消。
    Consumed,
    /// core 拒绝这次物理发送前授权；没有执行 native write。
    Authorize(AttemptError),
    /// 已收到 native 写入结果，但 core 无法回填它。
    Finish(AttemptError),
    /// native 写返回非 `WouldBlock` 错误；本次提交结果未知。
    Io(io::Error),
    /// native 写的结果未知，且 core 也拒绝记录未知状态。
    Abandon {
        /// 原始 SocketCAN I/O 错误。
        io: io::Error,
        /// core 回填错误。
        core: AttemptError,
    },
    /// `cancel_unsubmitted` 的 core 验证失败。
    Cancel(AttemptError),
    /// `abandon` 的 core 回填失败。
    AbandonCore(AttemptError),
}

impl fmt::Display for SocketCanTxError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Consumed => formatter.write_str("SocketCAN TX attempt was already consumed"),
            Self::Authorize(error) => {
                write!(formatter, "SocketCAN TX was not authorized: {error:?}")
            }
            Self::Finish(error) => {
                write!(formatter, "cannot record SocketCAN TX result: {error:?}")
            }
            Self::Io(error) => write!(formatter, "SocketCAN write has unknown result: {error}"),
            Self::Abandon { io, core } => write!(
                formatter,
                "SocketCAN write has unknown result ({io}); core recovery failed: {core:?}"
            ),
            Self::Cancel(error) => {
                write!(formatter, "cannot record SocketCAN cancellation: {error:?}")
            }
            Self::AbandonCore(error) => {
                write!(formatter, "cannot record unknown SocketCAN TX: {error:?}")
            }
        }
    }
}

impl std::error::Error for SocketCanTxError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(error) | Self::Abandon { io: error, .. } => Some(error),
            Self::Consumed
            | Self::Authorize(_)
            | Self::Finish(_)
            | Self::Cancel(_)
            | Self::AbandonCore(_) => None,
        }
    }
}

/// 一帧被 SocketCAN 接收端点读出的原生帧、时间戳和协议分类。
#[derive(Debug)]
pub struct CanRxFrame {
    /// 原始 SocketCAN 帧，含 Classic、RTR、FD 或总线错误通知。
    pub frame: CanAnyFrame,
    /// `SO_TIMESTAMPNS`、软件和硬件时间戳；每个字段保持 socketcan 4 的原始时钟域。
    pub timestamps: CanTimestamps,
    /// 调用方在原生时间戳可见后映射到 core 单调时钟域的接收事件时间。
    pub received_at: Instant,
    /// 协议分类；`Err` 表示原始帧是 Linux 错误通知。
    pub classification: Result<IngestResult, FromSocketcanError>,
}

/// 将已由应用读取的原生帧交给 core 分类。
///
/// 该函数不执行 I/O。所有 [`CanAnyFrame`] 变体和 [`CanTimestamps`] 都会原样归还；`Error`
/// 帧不是 ODrive 协议消息。
pub fn receive_frame(
    driver: &mut Driver<'_>,
    frame: CanAnyFrame,
    timestamps: CanTimestamps,
    received_at: Instant,
) -> CanRxFrame {
    let classification =
        FrameRef::try_from(&frame).map(|protocol_frame| driver.ingest(protocol_frame, received_at));
    CanRxFrame {
        frame,
        timestamps,
        received_at,
        classification,
    }
}
