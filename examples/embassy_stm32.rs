// Copyright The odrive-can-driver Contributors
//! 用已配置的 Embassy STM32 FDCAN 完成一个调用方已准备的 ODrive 操作。
//!
//! 应用先用 `Driver::prepare_command` 或 `Driver::prepare_query` 获得 `OperationId`，再调用
//! [`exchange_until`]。函数用原生异步 FDCAN 发送、持续接收，直到操作可由
//! `Driver::take_report` 提取。每个原生 TX/RX 结果立即交给调用方回调，因而无关的 Classic、
//! FD、RTR 与转换失败帧仍可由共享总线所有者分发。
//!
//! 若外层在 TX 等待中取消本 future，发送 guard 会将操作记为 `Unknown`；若在 RX 等待中取消，
//! 已提交查询仍为 `Submitted`。两种情况都保留调用方的 `OperationId`，可直接用
//! `Driver::report`、`Driver::tick`、`Driver::take_report` 和 `Driver::acknowledge_unknown`
//! 继续处置。本示例不会自动重发、清错或确认设备执行。

#![no_std]

use core::{
    future::{Future, poll_fn},
    task::Poll,
};

use embassy_stm32::can::{Can, enums::BusError, frame::FdEnvelope};
use odrive_can_driver::{
    BeginSendError, Driver, OperationId, OperationReport, OperationState, PrepareError,
    ReportError,
    embassy::{EmbassyReceive, EmbassyTransmit, receive_with_timestamp, transmit},
};

/// 一个已终结的操作。
#[derive(Debug)]
pub enum ExchangeResult {
    /// 调用方操作已经从 driver 取走，唯一操作槽位已释放。
    Complete(OperationReport),
    /// 发送结果未知；报告仍留在 driver，调用方必须先处置底层发送，再显式确认未知状态。
    Unknown(OperationReport),
}

/// 共享总线所有者接收原生 TX 与 RX 结果的回调。
pub struct Handlers<HandleTx, HandleRx> {
    /// 本地 FDCAN 写入完成后的结果，包含可能被替换的原生帧。
    pub tx: HandleTx,
    /// 每个从 FDCAN 出队的原生帧及其 driver 分类。
    pub rx: HandleRx,
}

/// 交换流程的局部错误；错误发生后 `id` 仍由调用方持有。
#[derive(Debug)]
pub enum ExchangeError {
    /// 调用方的时钟不能推进 core。
    Clock(PrepareError),
    /// core 拒绝开始 FDCAN 写入。
    Begin(BeginSendError),
    /// FDCAN 接收报告总线错误。
    Receive(BusError),
    /// `deadline` future 比 driver 的 `deadline_ms` 更早完成，无法诚实地宣布超时。
    DeadlineBeforeDriverDeadline {
        /// deadline future 完成时读取的单调时间。
        now_ms: u64,
        /// 调用方准备操作时声明的期限。
        deadline_ms: u64,
    },
    /// 操作标识不属于当前 driver，或不能被本流程继续推进。
    Report(ReportError),
    /// 一个外部可见但不应在没有未释放发送 guard 时出现的操作状态。
    UnexpectedState(OperationState),
}

enum TransmitWait {
    Transmitted(Result<EmbassyTransmit, BeginSendError>),
    Deadline,
}

enum ReceiveWait {
    Received(Result<EmbassyReceive, BusError>),
    Deadline,
}

fn take_if_finished(
    driver: &mut Driver,
    id: OperationId,
) -> Result<Option<ExchangeResult>, ExchangeError> {
    match driver.take_report(id) {
        Ok(report) => Ok(Some(ExchangeResult::Complete(report))),
        Err(ReportError::Pending) => Ok(None),
        Err(ReportError::UnknownPending) => driver
            .report(id)
            .map(ExchangeResult::Unknown)
            .map(Some)
            .map_err(ExchangeError::Report),
        Err(error) => Err(ExchangeError::Report(error)),
    }
}

/// 发送并持续接收一个调用方已准备的操作，直至其终结或 deadline 到达。
///
/// `deadline` 必须在该操作的 `deadline_ms` 对应时刻完成；函数会先轮询它，故已经到期时不会
/// 启动 I/O。若 deadline 在尚未首次轮询的 TX 前完成，操作保持 `Prepared` 并由 tick 变为
/// `TimedOut`；若在等待中的 TX 期间完成，TX future 析构会保守地将操作保留为 `Unknown`。
/// `now_ms` 与 `timestamp_ms` 都必须映射到 driver 的同一不回退单调毫秒时钟域。
pub async fn exchange_until<Clock, Deadline, Timestamp, HandleTx, HandleRx>(
    can: &mut Can<'_>,
    driver: &mut Driver,
    id: OperationId,
    mut now_ms: Clock,
    deadline: Deadline,
    mut timestamp_ms: Timestamp,
    handlers: Handlers<HandleTx, HandleRx>,
) -> Result<ExchangeResult, ExchangeError>
where
    Clock: FnMut() -> u64,
    Deadline: Future<Output = ()>,
    Timestamp: FnMut(&FdEnvelope) -> u64,
    HandleTx: FnMut(EmbassyTransmit),
    HandleRx: FnMut(EmbassyReceive),
{
    let Handlers {
        tx: mut handle_tx,
        rx: mut handle_rx,
    } = handlers;
    driver.tick(now_ms()).map_err(ExchangeError::Clock)?;
    if let Some(result) = take_if_finished(driver, id)? {
        return Ok(result);
    }

    let mut deadline = core::pin::pin!(deadline);
    match driver.report(id).map_err(ExchangeError::Report)?.state {
        OperationState::Prepared => {
            let transmit_wait = {
                let transmit = transmit(can, driver, id, &mut now_ms);
                let mut transmit = core::pin::pin!(transmit);
                poll_fn(|context| {
                    if deadline.as_mut().poll(context).is_ready() {
                        return Poll::Ready(TransmitWait::Deadline);
                    }
                    if let Poll::Ready(transmitted) = transmit.as_mut().poll(context) {
                        return Poll::Ready(TransmitWait::Transmitted(transmitted));
                    }
                    Poll::Pending
                })
                .await
            };
            match transmit_wait {
                TransmitWait::Transmitted(transmitted) => {
                    handle_tx(transmitted.map_err(ExchangeError::Begin)?);
                    if let Some(result) = take_if_finished(driver, id)? {
                        return Ok(result);
                    }
                }
                TransmitWait::Deadline => {
                    let deadline_now_ms = now_ms();
                    driver.tick(deadline_now_ms).map_err(ExchangeError::Clock)?;
                    if let Some(result) = take_if_finished(driver, id)? {
                        return Ok(result);
                    }
                    return Err(ExchangeError::DeadlineBeforeDriverDeadline {
                        now_ms: deadline_now_ms,
                        deadline_ms: driver
                            .report(id)
                            .map_err(ExchangeError::Report)?
                            .deadline_ms,
                    });
                }
            }
        }
        OperationState::Submitted => {}
        state => return Err(ExchangeError::UnexpectedState(state)),
    }

    loop {
        let receive_wait = {
            let receive = receive_with_timestamp(can, driver, |envelope| timestamp_ms(envelope));
            let mut receive = core::pin::pin!(receive);
            poll_fn(|context| {
                if deadline.as_mut().poll(context).is_ready() {
                    return Poll::Ready(ReceiveWait::Deadline);
                }
                if let Poll::Ready(received) = receive.as_mut().poll(context) {
                    return Poll::Ready(ReceiveWait::Received(received));
                }
                Poll::Pending
            })
            .await
        };

        match receive_wait {
            ReceiveWait::Received(received) => {
                handle_rx(received.map_err(ExchangeError::Receive)?);
                driver.tick(now_ms()).map_err(ExchangeError::Clock)?;
                if let Some(result) = take_if_finished(driver, id)? {
                    return Ok(result);
                }
            }
            ReceiveWait::Deadline => {
                let deadline_now_ms = now_ms();
                driver.tick(deadline_now_ms).map_err(ExchangeError::Clock)?;
                if let Some(result) = take_if_finished(driver, id)? {
                    return Ok(result);
                }
                return Err(ExchangeError::DeadlineBeforeDriverDeadline {
                    now_ms: deadline_now_ms,
                    deadline_ms: driver
                        .report(id)
                        .map_err(ExchangeError::Report)?
                        .deadline_ms,
                });
            }
        }
    }
}
