// Copyright The odrive-can-driver Contributors

//! ODrive CANSimple 的无分配共享总线驱动 core。
//!
//! 此 crate 只追踪本地发送与观察到的协议帧；它不保证设备执行命令，也不会自动重试、
//! 查询、清错或改变轴状态。
//!
//! The application owns the CAN endpoints. It calls [`Driver::authorize_tx`] before every
//! native TX poll, and retains native frames, FD/RTR distinctions and backend errors itself.

#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]

mod driver;

#[cfg(feature = "embedded-can")]
pub mod embedded_can;

#[cfg(feature = "embassy-stm32")]
pub mod embassy;

#[cfg(all(feature = "socketcan", target_os = "linux"))]
pub mod socketcan;

pub use driver::{
    AttemptError, BeginSendError, CachedResponse, Driver, Duration, IngestResult, Instant,
    OperationId, OperationKind, OperationReport, OperationState, PrepareError, ReportError,
    ResponseCache, ResponseCandidate, ResponseKind, SendPermit, Session, TxAttempt, TxCompletion,
    TxOutcome,
};
/// ODrive CANSimple 编解码器及硬件无关帧类型。
pub mod protocol;
