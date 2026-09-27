// Copyright The odrive-can-driver Contributors
//! Embassy STM32 的 FDCAN 原生异步适配。
//!
//! 本模块面向具有 FDCAN 的芯片，直接使用 Embassy 的 `Can::write` 与 `Can::read_fd`。
//! 应用负责配置外设、统一共享总线调度和为 `received_at_ms` 提供单调时钟。
//!
//! 消费方工作区根必须将 `embassy-stm32` patch 到 Embassy Git revision
//! `7b08a9c7d9a9fe620f4be25c4e7b86dd29f09f54`，配置见仓库 README。
//! crates.io 的 `embassy-stm32 0.6.0` 仅在 DLC 为 0 时设置 FDCAN RTR 位，而 CANSimple
//! 查询使用 DLC 8；该修订正确保留 RTR 位。Cargo 不向消费方传递 library 的 patch。

use embassy_stm32::can::{
    Can,
    enums::BusError,
    frame::{FdEnvelope, FdFrame, Frame},
};
use odrive_can_protocol::compat::embassy::FromEmbassyError;

use crate::{AttemptError, BeginSendError, Driver, IngestResult, OperationId};

/// FDCAN 发送的本地结果。
///
/// `Submitted` 只表示 Embassy 已将新帧交给本地队列。若 `displaced` 为 `Some`，该帧此前
/// 已由同一个共享控制器排队，但被本操作替换；调用方必须继续处理该原生帧。
#[derive(Debug)]
pub enum EmbassyTransmit {
    /// 新帧已提交，携带可能被替换的原生经典帧。
    Submitted {
        /// 被 FDCAN 队列替换的帧；该帧不属于本 driver 的所有权。
        displaced: Option<Frame>,
    },
    /// FDCAN 已接受新帧，但 core 无法以调用方给出的时钟确认提交状态。
    ///
    /// 操作 guard 已进入 `Unknown`；即使本变体仍保留 `displaced`，调用方也不得自动重试。
    Uncertain {
        /// 已被替换的原生帧，仍须交还共享总线调度器。
        displaced: Option<Frame>,
        /// core 拒绝记录提交的原因。
        error: AttemptError,
    },
}

/// Embassy 接收路径的结果。
#[derive(Debug)]
pub enum EmbassyReceive {
    /// 原生帧及 core 对它的协议分类。
    ///
    /// 即使分类为 `DecodeError` 或 `Unrelated`，原生 FDCAN 容器也保留给共享总线调用方。
    Frame {
        /// 原始 FDCAN 容器，保留 Classic、FD 和 RTR 标志。
        frame: FdFrame,
        /// 此 driver 的协议分类。
        classification: IngestResult,
    },
    /// 原生帧头部无法安全转换为协议视图，帧和原因一并归还。
    Invalid {
        /// 保持原始 FDCAN/Classic/RTR 标志的帧。
        frame: FdFrame,
        /// 转换失败原因。
        error: FromEmbassyError,
    },
}

/// 异步提交一个已准备的 Classic CANSimple 帧。
///
/// `now_ms` 在调用底层 `write` 前后分别读取，必须与 [`Driver::prepare_command`] 和
/// [`Driver::prepare_query`] 使用同一单调毫秒时钟。future 在 `write` 等待期间被取消时，
/// `SendAttempt` 析构会把本操作记为 `Unknown`，因为 FDCAN 可能已经接收该帧。若返回置换
/// 帧，新帧已经是 `Submitted`，置换帧仅供共享总线调用方另行处理。
pub async fn transmit(
    can: &mut Can<'_>,
    driver: &mut Driver,
    id: OperationId,
    mut now_ms: impl FnMut() -> u64,
) -> Result<EmbassyTransmit, BeginSendError> {
    let attempt = driver.begin_send(id, now_ms())?;
    let frame = Frame::from(attempt.frame());
    let displaced = can.write(&frame).await;
    match attempt.submitted(now_ms()) {
        Ok(()) => Ok(EmbassyTransmit::Submitted { displaced }),
        Err(error) => Ok(EmbassyTransmit::Uncertain { displaced, error }),
    }
}

/// 异步读取一帧 FDCAN 容器并交给共享 core 分类。
///
/// 使用 `read_fd` 而非 Classic-only `read`，以便无关的 FD、RTR 和 Classic 帧保持在
/// [`FdFrame`] 容器中返回。`received_at_ms` 在帧已出队后由 `timestamp_ms` 计算，调用方可
/// 将 `FdEnvelope::ts` 映射到自己的单调毫秒时钟。若使用主机观察时刻，它只表示出队观察，
/// 不能证明线上采样新鲜；本模块不会清空可能更早到达的 RX 队列。
pub async fn receive_with_timestamp(
    can: &mut Can<'_>,
    driver: &mut Driver,
    timestamp_ms: impl FnOnce(&FdEnvelope) -> u64,
) -> Result<EmbassyReceive, BusError> {
    let envelope = can.read_fd().await?;
    let received_at_ms = timestamp_ms(&envelope);
    Ok(ingest_fd_frame(driver, envelope.frame, received_at_ms))
}

/// 将一个由共享总线调用方拥有的 FDCAN 帧交给 core 分类。
///
/// 所有成功构造协议视图的帧都会连同原生容器返回；分类可能是 `Message`、`Unrelated` 或
/// `DecodeError`。因此协议解码在节点过滤前拒绝扩展 ID 或 FD 时，帧也不会被吞掉。
pub fn ingest_fd_frame(driver: &mut Driver, frame: FdFrame, received_at_ms: u64) -> EmbassyReceive {
    match odrive_can_protocol::FrameRef::try_from(&frame) {
        Ok(view) => EmbassyReceive::Frame {
            classification: driver.ingest(view, received_at_ms),
            frame,
        },
        Err(error) => EmbassyReceive::Invalid { frame, error },
    }
}
