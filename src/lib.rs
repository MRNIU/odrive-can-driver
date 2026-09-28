// Copyright The odrive-can-driver Contributors

//! ODrive CANSimple 的无分配共享总线驱动 core。
//!
//! 此 crate 只追踪本地发送与观察到的协议帧；它不保证设备执行命令，也不会自动重试、
//! 查询、清错或改变轴状态。
//!
//! ```
//! use odrive_can_driver::{Driver, OperationState, protocol::{Command, NodeId}};
//!
//! let mut driver = Driver::new(NodeId::new(1).unwrap());
//! let id = driver.prepare_command(Command::ClearErrors, 0, 10).unwrap();
//! let attempt = driver.begin_send(id, 1).unwrap();
//! assert_eq!(attempt.frame().data(), &[]);
//! attempt.submitted(1).unwrap();
//! let report = driver.take_report(id).unwrap();
//! assert_eq!(report.state, OperationState::Submitted);
//! ```

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
    AttemptError, BeginSendError, CachedResponse, Driver, IngestResult, OperationId, OperationKind,
    OperationReport, OperationState, PrepareError, ReportError, ResponseCache, ResponseKind,
    SendAttempt,
};
/// ODrive CANSimple 编解码器及硬件无关帧类型。
pub mod protocol;
