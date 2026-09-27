// Copyright The odrive-can-driver Contributors
//! 用 `embedded-can::nb::Can` 持续推进一个共享总线操作。
//!
//! 应用用 `Driver::prepare_command` 或 `Driver::prepare_query` 建立操作，再在自己的主循环中
//! 调用 [`poll_once`]。每轮调用都推进期限、取得至多一个 RX 帧，并在操作仍为 `Prepared` 时
//! 尝试 TX；高频 RX 流量不会阻止 TX 或终态报告。`received` 始终归调用方分发，库不清空 RX。
//!
//! `transmitted == Some(Ok(NbTransmit::WouldBlock))` 表示控制器明确没有接收该帧，下轮可
//! 继续；其他发送错误不证明未发。`report` 为 `Ok` 且其中状态为 `Unknown` 时，应用先在
//! 控制器层排除迟到发送，再直接调用 `Driver::acknowledge_unknown`，不得自动重发。完成报告由调用方直接以
//! `Driver::take_report` 取得。

#![no_std]

use embedded_can::{Frame, nb};
use odrive_can_driver::{
    Driver, OperationId, OperationReport, OperationState, PrepareError, ReportError,
    embedded_can::{
        NbReceive, NbReceiveError, NbTransmit, NbTransmitError, receive_nb, transmit_nb,
    },
};

/// 一次非阻塞总线调度的完整观察结果。
#[derive(Debug)]
pub struct Poll<F, E> {
    /// 本轮取得的一帧，或控制器队列为空；原生帧由调用方继续分发。
    pub received: NbReceive<F>,
    /// 仅当操作在本轮进入发送时存在；发送错误与已接收原生帧可同时被调用方观察。
    pub transmitted: Option<Result<NbTransmit<F>, NbTransmitError<E>>>,
    /// 在 RX、TX 与期限推进后的操作报告；即使报告读取失败，`received` 仍已归还。
    pub report: Result<OperationReport, ReportError>,
}

/// 开始本轮调度前无法继续的局部错误。
#[derive(Debug)]
pub enum PollError<F, E> {
    /// 调用方的单调时钟无法推进 core；此检查发生在读取 RX 前。
    Clock(PrepareError),
    /// 接收路径的控制器或 Classic 帧转换错误；转换失败时原始帧仍在错误中。
    Receive(NbReceiveError<F, E>),
    /// 操作标识不属于当前 driver。
    Report(ReportError),
}

/// [`poll_once`] 的返回类型，保留收到的原生帧、发送结果和报告读取错误。
pub type PollResult<F, E> = Result<Poll<F, E>, PollError<F, E>>;

/// 推进一个已准备的操作一次。
///
/// `now_ms` 会在 tick、RX 时间戳和 TX 前后分别读取，必须始终返回同一不回退的单调毫秒
/// 时钟域。函数在 RX 之后不再以 `Result` 丢弃已收到帧；发送结果保留在 [`Poll::transmitted`]。
/// 此函数不提取完成报告，调用方可先处理 `received`，再决定何时调用 `Driver::take_report`。
pub fn poll_once<C>(
    driver: &mut Driver,
    id: OperationId,
    can: &mut C,
    mut now_ms: impl FnMut() -> u64,
) -> PollResult<C::Frame, C::Error>
where
    C: nb::Can,
    C::Frame: Frame,
{
    driver.tick(now_ms()).map_err(PollError::Clock)?;
    let should_transmit =
        driver.report(id).map_err(PollError::Report)?.state == OperationState::Prepared;
    let received = receive_nb(driver, can, &mut now_ms).map_err(PollError::Receive)?;
    let transmitted = should_transmit.then(|| transmit_nb(driver, id, can, &mut now_ms));
    let report = driver.report(id);

    Ok(Poll {
        received,
        transmitted,
        report,
    })
}
