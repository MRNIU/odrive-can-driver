// Copyright The odrive-can-protocol Contributors

//! 原生 CAN 帧适配，保持标识符、有效长度与帧类型，不执行任何 I/O。

/// Embassy STM32 0.6 的 `From`／`TryFrom` 帧转换；芯片由应用选择。
#[cfg(feature = "embassy-stm32")]
pub mod embassy;
/// `embedded-can` 0.4 的通用 Classic CAN 转换。
pub mod embedded_can;
/// Linux SocketCAN 4 的 `From`／`TryFrom` 帧转换。
#[cfg(all(feature = "socketcan", target_os = "linux"))]
pub mod socketcan;
