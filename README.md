<!-- Copyright The odrive-can-driver Contributors -->
<!-- 本文件说明驱动的产品边界、兼容范围、安装与操作结果语义。 -->

# odrive-can-driver

[![CI](https://github.com/MRNIU/odrive-can-driver/actions/workflows/ci.yml/badge.svg)](https://github.com/MRNIU/odrive-can-driver/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/odrive_can_driver.svg)](https://crates.io/crates/odrive_can_driver)
[![docs.rs](https://docs.rs/odrive_can_driver/badge.svg)](https://docs.rs/odrive_can_driver)
[![MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/MRNIU/odrive-can-driver/blob/main/LICENSE)
[![MSRV](https://img.shields.io/badge/MSRV-1.85-blue.svg)](https://blog.rust-lang.org/2025/02/20/Rust-1.85.0/)

`odrive_can_driver`（仓库名 `odrive-can-driver`）是面向 ODrive CANSimple 的无分配驱动库。它在 `odrive-can-protocol 0.1.2` 的编解码之上记录单节点操作的本地提交、观察到的查询反馈、超时和副作用未知状态，并提供 `embedded-can`、Embassy STM32 与 Linux SocketCAN 的真实 I/O 后端。

默认构建使用 Rust 2024、`#![no_std]`、无 `alloc`，仅依赖 `odrive-can-protocol 0.1.2`；核心 MSRV 为 Rust 1.85。该库不配置 CAN 芯片、时钟、引脚或位速率，也不决定机械限值、保护许可和设备恢复策略。

## 安装与后端

Rust 中的 crate 名为 `odrive_can_driver`，协议类型从 `odrive_can_driver::protocol` 导入。完整 API、参数前提、毫秒时钟和每个错误的副作用语义见 [rustdoc](https://docs.rs/odrive_can_driver)。

```toml
[dependencies]
odrive_can_driver = "0.1.0"
```

| feature | 最小依赖配置 | 真实 I/O 与运行环境 |
|---|---|---|
| `embedded-can` | `odrive_can_driver = { version = "0.1.0", features = ["embedded-can"] }` | `embedded-can 0.4` 的 blocking 与 `nb` 接口；调用方拥有外设和总线调度。 |
| `embassy-stm32` | `odrive_can_driver = { version = "0.1.0", features = ["embassy-stm32"] }`，另加名义 `0.6.0` 的 `embassy-stm32` 及下文固定 revision patch | 原生异步 FDCAN；首版以 STM32H723 为目标，应用选择芯片、时钟、引脚与 CAN 配置；当前最低已验证 Rust 1.98.1。 |
| `socketcan` | `odrive_can_driver = { version = "0.1.0", features = ["socketcan"] }`；下例直接配置错误过滤器还需 `socketcan = { version = "4", default-features = false }` | Linux `socketcan 4` 的非阻塞套接字，不引入 Tokio 或其他运行时；MSRV 为 Rust 1.89。 |

Embassy 的芯片 feature 由消费方选择；表中的 H723 仅是当前目标组合。SocketCAN 仅在 Linux 上可用。每个后端的最小可运行消费方、设备前提和命令见 [examples/README.md](examples/README.md)；Linux 可直接运行一次 [`socketcan_query`](examples/socketcan_query.rs) 母线电压 RTR 查询。

### Embassy STM32H723：完整 RTR 可用配置

Embassy 依赖在消费方工作区根声明为名义 `0.6.0`，并且**必须**以 EHA 已使用的固定 revision 覆盖六个包。Cargo 发布不会把本库的 `[patch.crates-io]` 传递给下游，因此只启用本库的 `embassy-stm32` feature 不是完整 RTR 可用组合；将下列内容复制到消费方工作区根的 `Cargo.toml`：

```toml
[dependencies]
odrive_can_driver = { version = "0.1.0", features = ["embassy-stm32"] }
embassy-stm32 = { version = "0.6.0", features = ["stm32h723vg"] }

[patch.crates-io]
embassy-stm32 = { git = "https://github.com/embassy-rs/embassy", rev = "7b08a9c7d9a9fe620f4be25c4e7b86dd29f09f54" }
embassy-executor = { git = "https://github.com/embassy-rs/embassy", rev = "7b08a9c7d9a9fe620f4be25c4e7b86dd29f09f54" }
embassy-time = { git = "https://github.com/embassy-rs/embassy", rev = "7b08a9c7d9a9fe620f4be25c4e7b86dd29f09f54" }
embassy-sync = { git = "https://github.com/embassy-rs/embassy", rev = "7b08a9c7d9a9fe620f4be25c4e7b86dd29f09f54" }
embassy-usb = { git = "https://github.com/embassy-rs/embassy", rev = "7b08a9c7d9a9fe620f4be25c4e7b86dd29f09f54" }
embassy-futures = { git = "https://github.com/embassy-rs/embassy", rev = "7b08a9c7d9a9fe620f4be25c4e7b86dd29f09f54" }
```

crates.io 的 `embassy-stm32 0.6.0` 未包含此修复：FDCAN 发送 RTR 时仅在 `header.len() == 0` 设置 RTR 位，而 CANSimple RTR 查询使用 DLC 8，结果会被发送为数据帧，ODrive `fw-v0.5.1` 不会返回查询回复。固定 revision 使用 `header.rtr()` 保留该 RTR 位。不要把 registry `0.6.0` 单独称为完整 RTR 可用配置。

固定 Git Embassy H723 组合当前以 Rust 1.98.1 验证；这不是已穷尽版本二分的真实 MSRV 声明。Rust 1.89 的 target check 当前会在传递依赖 `xarxa-driver` 的 `cfg_select!` 处失败。该限制独立于默认/`embedded-can` 的 Rust 1.85 与 SocketCAN 的 Rust 1.89 支持。

[docs.rs](https://docs.rs/odrive_can_driver) 只展示公开 API；它不验证真实总线行为，也不替消费方工作区应用上述 patch。

已准备的 `id`、调用方拥有的 `driver` 和同一单调时钟域的时刻是下列后端调用共同前提。后端只执行一次真实 I/O，并把本地结果回写给 `Driver`；发送端以外的总线策略仍由应用持有。

```rust
use odrive_can_driver::embedded_can::{NbReceive, NbTransmit, receive_nb, transmit_nb};

match transmit_nb(&mut driver, id, &mut can, || monotonic_ms())? {
    NbTransmit::Submitted { displaced } => hand_back_to_bus_owner(displaced),
    NbTransmit::WouldBlock => schedule_same_operation_again(id),
    NbTransmit::Uncertain { displaced, error } => {
        hand_back_to_bus_owner(displaced);
        record_unknown(id, error);
    }
}
match receive_nb(&mut driver, &mut can, || monotonic_ms())? {
    NbReceive::Frame { frame, classification } => {
        record_protocol_result(classification);
        hand_back_to_bus_owner(Some(frame));
    }
    NbReceive::Empty => {}
}
```

`embedded_can::transmit_blocking` 和 `receive_blocking` 提供同一状态语义的 blocking 入口；blocking 调用没有标准 trait 提供的可取消期限，不能把调用取消当作未发送。`transmit_nb` 的原生 I/O 错误也不能证明帧未提交，因此 guard 析构后为 `Unknown`。

```rust
use odrive_can_driver::embassy::{EmbassyReceive, receive_with_timestamp, transmit};

let tx = transmit(&mut fdcan, &mut driver, id, || monotonic_ms()).await?;
handle_embassy_transmit(tx);
let rx = receive_with_timestamp(&mut fdcan, &mut driver, timestamp_to_ms).await?;
match rx {
    EmbassyReceive::Frame { frame, classification } => {
        record_protocol_result(classification);
        hand_back_to_bus_owner(Some(frame));
    }
    EmbassyReceive::Invalid { frame, error } => {
        record_frame_error(error);
        hand_back_to_bus_owner(Some(frame));
    }
}
```

`fdcan` 是应用已经配置好的 `embassy_stm32::can::Can`；`transmit` 的闭包在异步写入前后读取同一单调毫秒时钟。`timestamp_to_ms` 必须将 `FdEnvelope` 的硬件时间映射到该时钟域。

```rust
use odrive_can_driver::socketcan::SocketCan;
use socketcan::{SocketOptions, id::ERR_MASK_ALL};

let bus = SocketCan::open("can0")?;
bus.socket().set_error_filter(ERR_MASK_ALL)?;
bus.send(&mut driver, id, || monotonic_ms())?;
let received = bus.receive(&mut driver, || monotonic_ms())?;
record_socketcan_classification(received.classification);
hand_back_to_bus_owner(Some(received.frame));
```

`SocketCan::open` 只打开并设为非阻塞模式；它不会创建接口、配置位速率或修改过滤器。SocketCAN 默认禁用错误通知；应用若需要保留错误帧，须通过 `bus.socket().set_error_filter(...)` 显式配置该描述符。`send` 遇到 `WouldBlock` 时明确未入队并回到 `Prepared`；其他 I/O 错误会保守地留下 `Unknown`，因为它们不能证明帧没有离开进程。

## 最小操作流程

一个 `Driver` 绑定一个节点，并只保留一个活动或一个尚未取走的终态报告。应用以统一的单调毫秒时钟调用 `prepare_command` 或 `prepare_query`，取得 `OperationId` 后以 `begin_send` 取得 `SendAttempt`。应用把 `attempt.frame()` 交给自己拥有的发送路径，并依实际结果调用 `submitted`、`would_block` 或 `not_sent`；终态后先以 `take_report` 取走报告才可准备下一操作。查询反馈由应用在接收和分发时调用 `ingest`；周期性调用 `tick` 处理期限。

```rust,no_run
use odrive_can_driver::{Driver, OperationState};
use odrive_can_driver::protocol::{NodeId, Query};

let mut driver = Driver::new(NodeId::new(7).unwrap());
let id = driver.prepare_query(Query::MotorError, 0, 100).unwrap();
let attempt = driver.begin_send(id, 1).unwrap();

// 将 attempt.frame() 交给当前后端；确认其已接收该帧后才提交。
let _encoded = attempt.frame();
attempt.submitted(1).unwrap();

// 在统一 RX 分发处调用 driver.ingest(frame, rx_ms)，并定期推进期限。
driver.tick(100).unwrap();
assert_eq!(driver.take_report(id).unwrap().state, OperationState::TimedOut);
```

该片段刻意只展示所有后端共有的操作边界；可编译的收发接入见各示例。不得以 Heartbeat、同类周期帧或其他主机的查询回复证明写命令完成。

仓库中的 [`shared_bus`](examples/shared_bus.rs) 以默认 feature 演示“调用方发送并回报已知结果”的最小边界：

```sh
cargo run --example shared_bus
```

## 操作结果与副作用

`OperationReport` 保留操作 ID、种类、期限、准备、发送中、本地提交和终态时间戳。设备 CANSimple 没有事务序号，因此报告将“已经发送”和“设备已经执行”严格分开。

| 情况或状态 | 含义 | 应用应做什么 |
|---|---|---|
| 未调用 | 没有准备操作、没有操作 ID，也没有发送副作用。 | 仅在产品许可成立时开始新的操作。 |
| 本地参数或时间错误 | `prepare_*` 在分配 ID 前拒绝请求，未形成帧。 | 修正调用参数；这不是设备错误，也没有可重试的设备副作用。 |
| `Prepared` / `Dispatching` | 已创建操作；`Dispatching` 表示发送已开始，尚不能证明底层未接收。 | 继续处理同一个 `SendAttempt`；不要并发启动另一操作。 |
| `Submitted` | 本地 I/O 确认接收了帧。写命令在此终态；查询仍等待反馈。 | 把它视为本地提交，不要声称设备已执行。 |
| `Observed` | 查询在提交后、期限前观察到同节点同类反馈。 | 把它视为观察到的协议反馈，不把它提升为带事务身份的设备 ACK。 |
| `Failed` | 后端或调用方以 `not_sent` 明确结束本次发送，已知帧未被本地提交。 | 读取报告并按产品策略处理；它不同于无法判定是否发送的 `Unknown`。 |
| `TimedOut` | 到达期限仍未完成；报告保留此前的发送进展。 | 按产品策略决定后续动作；不要自动重发副作用未知的命令。 |
| `Cancelled` | 仅在 `Prepared` 时由 `cancel` 取消，尚未进入发送。 | 可安全放弃该准备操作。 |
| `Unknown` | `SendAttempt` 在发送中被丢弃，或后端无法证明帧没有被接收。 | 先确认底层不再可能迟到发送，再 `acknowledge_unknown` 取走报告；不得自动重试。 |

`would_block` 只适用于后端明确尚未接收帧的情况，并将操作回到可继续发送的准备状态；`not_sent` 只用于已知未提交的帧，并记录终态 `Failed`。若这两个回调的 `now_ms` 回退，调用会返回时钟诊断：`would_block` 保持 `Prepared`，`not_sent` 以最后已知单调时刻记录 `Failed`，都不把已知未发送误报为 `Unknown`。取消 future、普通 I/O 错误或调用栈退出均不能单独证明帧未发送，必须保守报告 `Unknown`。

## 共享总线、watchdog 与控制边界

调用方拥有共享 CAN 总线的读取、分发和发送所有权。驱动不会清空 RX、重配或重建外设，也不会吞掉其他节点的帧；后端保留实际接收帧与可能被替换的待发送帧，供调用方统一分发和处置。

ODrive `fw-v0.5.1` 在有效节点的每一寻址 CANSimple 帧上喂 watchdog，包含 RTR。因而任意命令或查询都可能延长设备的 CAN watchdog；驱动绝不为此隐式发 query、Heartbeat 或保活帧。它同样不会自动重试、清除错误、请求闭环、恢复目标或判定运动完成。这些都是产品层必须显式作出的决定。

## 协议支持

本 crate 仅支持 `odrive-can-protocol 0.1.2` 已登记的 CANSimple 布局。它不因后端增加额外 ODrive 固件版本。

| 固件 | 协议模块 | 状态 |
|---|---|---|
| ODrive `fw-v0.5.1` | `odrive_can_driver::protocol::fw_v0_5_1` | 支持 |
| MKS ODrive Mini `ODriveMINI-fw-v0.5.1-20250326` | `odrive_can_driver::protocol::fw_v0_5_1` | 支持；与上述固定官方基线使用同一 CANSimple 布局。 |

MKS 结论仅适用于[协议库 README 指定的固定源码包](https://github.com/makerbase-motor/MKS-ODrive/blob/e15782976ae93d42b1f0648ceec96503141a343b/Firmware/MKS%20ODrive%20MINI/ODriveMINI-fw-v0.5.1-20250326.rar)及其核对的[官方 ODrive 固定 revision `7831d795`](https://github.com/odriverobotics/ODrive/tree/7831d795235e5ef8535e4b46621a0721b458ec8f)。完整协议依据见 [odrive-can-protocol README](https://github.com/MRNIU/odrive-can-protocol/blob/main/README.md)。其他 MKS 固件版本必须先核对源码，不能据此推断兼容性。

## 验证边界

CI 覆盖受控的软件构建、单元测试、文档、lint、嵌入式交叉编译和显式运行的 Linux vcan 集成测试。软件注入和 vcan 只证明相应软件路径，不能证明物理 Bus-Off、接线、电源、设备身份、ODrive 配置或机械动作。真实设备验证需要在连接设备的主机上按示例的门禁执行并单独记录证据。

## 贡献与许可

贡献要求、验证矩阵和发布步骤见 [CONTRIBUTING.md](CONTRIBUTING.md)。本项目采用 [MIT License](LICENSE)，并保留适用版权信息。
