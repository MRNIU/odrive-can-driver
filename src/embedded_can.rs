// Copyright The odrive-can-driver Contributors
//! `embedded-can` 0.4 Classic CAN 的同步与非阻塞适配。
//!
//! 此模块只接受 Classic 数据帧或 RTR。`embedded_can::Frame` 不表达 FDF 或总线错误位；需要
//! 保留这些信息的应用应在原生控制器层完成分发。调用方仍拥有共享总线的 RX/TX 调度权。

use crate::protocol::compat::embedded_can::InvalidFrameLength;
use embedded_can::{Frame, blocking, nb};

use crate::{AttemptError, BeginSendError, Driver, IngestResult, OperationId};

/// 非阻塞发送完成时的可观察结果。
#[derive(Debug, PartialEq, Eq)]
pub enum NbTransmit<F> {
    /// 新帧已被本地队列接受。
    Submitted {
        /// 被控制器替换的低优先级原生帧；调用方必须继续处理它。
        displaced: Option<F>,
    },
    /// 控制器确认未接收新帧；操作保持 `Prepared`，调用方可稍后重试。
    WouldBlock,
    /// 原生帧已入队，但 core 无法记录为 `Submitted`；操作已是 `Unknown`。
    ///
    /// 即使状态不确定，队列替换出的帧仍会原样返回。
    Uncertain {
        /// 被替换的原生帧。
        displaced: Option<F>,
        /// core 拒绝记录提交的原因。
        error: AttemptError,
    },
}

/// 非阻塞发送无法产生 [`NbTransmit`] 的原因。
#[derive(Debug)]
pub enum NbTransmitError<E> {
    /// core 拒绝开始本次发送。
    Begin(BeginSendError),
    /// 原生 Classic 帧构造器拒绝了已编码帧；操作已确定为 `Failed`。
    FrameRejected,
    /// 明确未发送或本地提交记录的时钟检查失败。
    Attempt(AttemptError),
    /// 控制器报告的非 `WouldBlock` 错误。
    ///
    /// `SendAttempt` 会析构成 `Unknown`，因为该错误不能证明帧未入队。
    Driver(E),
}

/// 非阻塞接收的可观察结果。
#[derive(Debug)]
pub enum NbReceive<F> {
    /// 当前没有可读帧。
    Empty,
    /// 原生帧及 core 对它的协议分类。
    ///
    /// 即使分类为 `DecodeError` 或 `Unrelated`，原生帧也保留给共享总线调用方。
    Frame {
        /// 原始 Classic CAN 或 RTR 帧。
        frame: F,
        /// 此 driver 的协议分类。
        classification: IngestResult,
    },
}

/// 非阻塞接收无法产生 [`NbReceive`] 的原因。
#[derive(Debug)]
pub enum NbReceiveError<F, E> {
    /// 原生驱动报告错误。
    Driver(E),
    /// 驱动帧不满足 Classic CAN 长度合同，原始帧没有被丢弃。
    InvalidFrame {
        /// 原始驱动帧。
        frame: F,
        /// 长度拒绝原因。
        error: InvalidFrameLength,
    },
}

/// `embedded-can::nb::Can` 接收一次的完整结果。
pub type NbReceiveResult<C> = Result<
    NbReceive<<C as nb::Can>::Frame>,
    NbReceiveError<<C as nb::Can>::Frame, <C as nb::Can>::Error>,
>;

/// blocking 发送无法明确提交的原因。
#[derive(Debug)]
pub enum BlockingTransmitError<E> {
    /// core 拒绝开始本次发送。
    Begin(BeginSendError),
    /// 原生 Classic 帧构造器拒绝了已编码帧；操作已确定为 `Failed`。
    FrameRejected,
    /// core 无法以调用方时钟记录发送结果。
    Attempt(AttemptError),
    /// blocking 驱动返回错误。
    ///
    /// 调用期间没有本模块可保证的 deadline 或取消点；guard 析构后操作为 `Unknown`。
    Driver(E),
}

/// blocking 接收无法产生 [`NbReceive`] 的原因。
#[derive(Debug)]
pub enum BlockingReceiveError<F, E> {
    /// 原生驱动报告错误。
    Driver(E),
    /// 驱动帧不满足 Classic CAN 长度合同，原始帧没有被丢弃。
    InvalidFrame {
        /// 原始驱动帧。
        frame: F,
        /// 长度拒绝原因。
        error: InvalidFrameLength,
    },
}

/// `embedded-can::blocking::Can` 接收一次的完整结果。
pub type BlockingReceiveResult<C> = Result<
    NbReceive<<C as blocking::Can>::Frame>,
    BlockingReceiveError<<C as blocking::Can>::Frame, <C as blocking::Can>::Error>,
>;

/// 尝试把一个已准备操作交给 `embedded-can::nb::Can`。
///
/// `Ok(Some(displaced))` 说明新帧已经进入队列，故本函数会先记录本操作 `Submitted`，再把
/// `displaced` 返回给调用方；它绝不会把置换帧当作本操作未发送。明确 `WouldBlock` 才会让
/// core 回到 `Prepared`。其他驱动错误和调用取消均保守地保留 `Unknown`。
pub fn transmit_nb<C>(
    driver: &mut Driver,
    id: OperationId,
    can: &mut C,
    mut now_ms: impl FnMut() -> u64,
) -> Result<NbTransmit<C::Frame>, NbTransmitError<C::Error>>
where
    C: nb::Can,
    C::Frame: Frame,
{
    let attempt = driver
        .begin_send(id, now_ms())
        .map_err(NbTransmitError::Begin)?;
    let Some(frame) = attempt.frame().to_embedded_can::<C::Frame>() else {
        attempt
            .not_sent(now_ms())
            .map_err(NbTransmitError::Attempt)?;
        return Err(NbTransmitError::FrameRejected);
    };
    match can.transmit(&frame) {
        Ok(displaced) => match attempt.submitted(now_ms()) {
            Ok(()) => Ok(NbTransmit::Submitted { displaced }),
            Err(error) => Ok(NbTransmit::Uncertain { displaced, error }),
        },
        Err(::nb::Error::WouldBlock) => attempt
            .would_block(now_ms())
            .map(|()| NbTransmit::WouldBlock)
            .map_err(NbTransmitError::Attempt),
        Err(::nb::Error::Other(error)) => Err(NbTransmitError::Driver(error)),
    }
}

/// 从 `embedded-can::nb::Can` 读取一帧并交给 core 分类。
///
/// `timestamp_ms` 在成功取出该帧后记录调用方观察时刻；控制器 RX 队列可能已经包含更早到达
/// 的帧，本函数不把此时刻宣称为物理线上采样时间，也不清空队列。
pub fn receive_nb<C>(
    driver: &mut Driver,
    can: &mut C,
    timestamp_ms: impl FnOnce() -> u64,
) -> NbReceiveResult<C>
where
    C: nb::Can,
    C::Frame: Frame,
{
    match can.receive() {
        Ok(frame) => ingest_classic_frame(driver, frame, timestamp_ms())
            .map_err(|(frame, error)| NbReceiveError::InvalidFrame { frame, error }),
        Err(::nb::Error::WouldBlock) => Ok(NbReceive::Empty),
        Err(::nb::Error::Other(error)) => Err(NbReceiveError::Driver(error)),
    }
}

/// 把一个已由调用方取得的 Classic CAN 帧交给 core 分类。
pub fn ingest_classic_frame<F>(
    driver: &mut Driver,
    frame: F,
    received_at_ms: u64,
) -> Result<NbReceive<F>, (F, InvalidFrameLength)>
where
    F: Frame,
{
    let view = match crate::protocol::FrameRef::from_classic_embedded_can(&frame) {
        Ok(view) => view,
        Err(error) => return Err((frame, error)),
    };
    let classification = driver.ingest(view, received_at_ms);
    Ok(NbReceive::Frame {
        frame,
        classification,
    })
}

/// 通过 `embedded-can::blocking::Can` 发送一个已准备操作。
///
/// 此适配没有标准 trait 提供的 deadline 或可撤销点。调用取消或 `Driver` 未能记录回调时，
/// 操作会保守地变为 `Unknown`；应用若需要可轮询的时效和取消语义，应使用 [`transmit_nb`]。
pub fn transmit_blocking<C>(
    driver: &mut Driver,
    id: OperationId,
    can: &mut C,
    mut now_ms: impl FnMut() -> u64,
) -> Result<(), BlockingTransmitError<C::Error>>
where
    C: blocking::Can,
    C::Frame: Frame,
{
    let attempt = driver
        .begin_send(id, now_ms())
        .map_err(BlockingTransmitError::Begin)?;
    let Some(frame) = attempt.frame().to_embedded_can::<C::Frame>() else {
        attempt
            .not_sent(now_ms())
            .map_err(BlockingTransmitError::Attempt)?;
        return Err(BlockingTransmitError::FrameRejected);
    };
    can.transmit(&frame)
        .map_err(BlockingTransmitError::Driver)?;
    attempt
        .submitted(now_ms())
        .map_err(BlockingTransmitError::Attempt)
}

/// 从 `embedded-can::blocking::Can` 读取一帧并交给 core 分类。
///
/// 成功返回时 `timestamp_ms` 给出调用方观察到出队帧的时刻；该帧可能已在硬件队列中等待。
pub fn receive_blocking<C>(
    driver: &mut Driver,
    can: &mut C,
    timestamp_ms: impl FnOnce() -> u64,
) -> BlockingReceiveResult<C>
where
    C: blocking::Can,
    C::Frame: Frame,
{
    let frame = can.receive().map_err(BlockingReceiveError::Driver)?;
    ingest_classic_frame(driver, frame, timestamp_ms())
        .map_err(|(frame, error)| BlockingReceiveError::InvalidFrame { frame, error })
}
