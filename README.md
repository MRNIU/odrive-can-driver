<!-- Copyright The odrive-can-driver Contributors -->
<!-- 说明安装、常用操作和异常处理；后端接入与时序统一放在 examples。 -->

# odrive-can-driver

[![CI](https://github.com/MRNIU/odrive-can-driver/actions/workflows/ci.yml/badge.svg)](https://github.com/MRNIU/odrive-can-driver/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/odrive_can_driver.svg)](https://crates.io/crates/odrive_can_driver)
[![docs.rs](https://docs.rs/odrive_can_driver/badge.svg)](https://docs.rs/odrive_can_driver)
[![MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![MSRV](https://img.shields.io/badge/MSRV-1.85-blue.svg)](https://blog.rust-lang.org/2025/02/20/Rust-1.85.0/)

`odrive_can_driver` 是 ODrive CANSimple 驱动库。共享 `Driver` 管理单节点的命令、查询、反馈缓存、操作身份、期限和发送结果；三个可选后端负责实际收发。默认 `no_std`、无动态分配，使用已发布的 `odrive-can-protocol 0.1.2` 编解码与帧适配。

## 安装

```toml
[dependencies]
odrive_can_driver = { version = "0.1.0", features = ["embedded-can"] }
```

按接入方式选择 feature；只使用共享状态机时可省略 `features`，默认是 `[]`。

| feature | 接入方式 | Rust 要求 | 使用入口 |
|---|---|---|---|
| `embedded-can` | 实现 `embedded-can 0.4` 的 HAL，blocking 或非阻塞轮询 | 1.85 | [轮询示例与时序](examples/README.md#embedded-can) |
| `embassy-stm32` | 已配置的 STM32 FDCAN，原生异步收发 | 固定依赖组合已验证 1.98.1 | [依赖配置、异步示例与时序](examples/README.md#embassy-stm32) |
| `socketcan` | Linux 非阻塞 SocketCAN，无异步运行时 | 1.89 | [命令行示例与时序](examples/README.md#socketcan) |

**Embassy 必须使用示例指南中的 workspace patch**：registry `embassy-stm32 0.6.0` 存在 RTR DLC 8 发送缺陷；已验证的 Git revision 修复了它。Cargo 不会向消费方传递本库的 patch。

## 一次操作怎样完成

1. 为设备节点创建 `Driver::new(NodeId::new(node)?)`。
2. 用 `prepare_command` 或 `prepare_query` 准备操作，取得 `OperationId`。这一步只验证和编码，没有发送。
3. 调用对应后端的发送函数；它内部完成 `begin_send` 和发送结果记录。
4. 持续接收并分发帧，同时用 `tick(now_ms)` 推进期限。接收会更新缓存，匹配的查询反馈可使操作成为 `Observed`。
5. 用 `report(id)` 查看进展；终态后 `take_report(id)` 取走报告，再开始下一操作。`Unknown` 有额外处理要求，见下文。

一个 `Driver` 只有一个操作槽，**未取走的终态报告也占用它**。所有时刻以同一单调时钟的毫秒值表示，`deadline_ms` 是绝对期限。例如当前 `1000`、期限 `1100` 表示剩余 100 ms；反馈必须在提交后且严格早于期限才参与查询完成判断。

## 发心跳、读状态与保活

**Heartbeat 由 ODrive 周期发送，主机接收。** 当前协议没有 `Command::Heartbeat` 或 `Query::Heartbeat`；本库不伪造设备心跳，也不配置设备心跳周期。先在设备端配置周期，随后持续调用后端接收函数，读取缓存中的轴状态、完整错误位和接收时间：

```rust
use odrive_can_driver::{Driver, ResponseKind, protocol::{AxisState, Response}};

fn fresh_axis_state(driver: &Driver, now_ms: u64, max_age_ms: u64)
    -> Option<(AxisState, u32)>
{
    if !driver.cache().heartbeat_is_fresh(now_ms, max_age_ms) {
        return None;
    }
    match driver.cache().get(ResponseKind::Heartbeat)?.response {
        Response::Heartbeat { axis_state, axis_error } => Some((axis_state, axis_error)),
        _ => None,
    }
}
```

`None` 表示没有可用的新鲜状态，不能当作 Idle 或无错误。非零 `axis_error` 是设备错误位图；未知位也保留。`max_age_ms` 由应用根据设备发送周期和允许的延迟设置。

如果“发心跳”指**主机保活**，应用可以按自己的调度周期调用 `prepare_query(Query::VbusVoltage, now_ms, deadline_ms)`，再通过后端真正发出并处理报告。在支持的 v0.5.1 固件中，寻址到该轴的 CANSimple 帧（包含 RTR 查询）会喂 watchdog。因此查询也会延长 watchdog，不能把持续查询后的存活误认为控制任务仍健康。库不创建后台保活任务；查询周期和 watchdog 超时由应用明确配置，上一操作结束并取走报告后才准备下一次。

## 发指令、查询数据、清错误

下面的值直接来自 `odrive_can_driver::protocol`。写指令传给 `prepare_command`，读取请求传给 `prepare_query`，两者随后都使用同一个后端发送入口。

| 目的 | 传入值 | 如何确认后续状态 |
|---|---|---|
| 请求停止输出/Idle | `Command::SetAxisRequestedState { state: AxisState::IDLE }` | 继续接收，要求提交后新的 Heartbeat 显示 Idle；机械停止还需应用自己的反馈判据 |
| 请求闭环 | `Command::SetAxisRequestedState { state: AxisState::CLOSED_LOOP_CONTROL }` | 在应用许可、标定和目标已准备的前提下发送，再观察新的状态与错误 |
| 设置速度 | `Command::SetInputVel { velocity, torque_ff }` | 单位分别为 turn/s、N·m；读 `EncoderEstimates` 观察实际运动 |
| 设置位置 | `Command::SetInputPos { position, velocity_ff, torque_ff }` | position 为 turn；两个前馈是原始 `i16`，每计数分别为 0.001 turn/s、0.001 N·m |
| 设置转矩 | `Command::SetInputTorque { torque }` | 单位 N·m；不隐式切换控制模式 |
| 设置控制/输入模式 | `Command::SetControllerModes { control_mode, input_mode }` | 两个字段是固件定义的原始模式值，应用选择适用组合 |
| 清错误 | `Command::ClearErrors` | 观察新的 Heartbeat，并按需重新查询各子系统错误；故障原因仍存在时可能再次报错 |
| 急停故障请求 | `Command::Estop` | 观察新的设备错误/状态；它仍依赖通信，不能替代独立硬件停止通道 |
| 读轴状态/轴错误 | 接收 `Response::Heartbeat` | 用上面的缓存时效检查，不发送虚构的 Heartbeat RTR |
| 读位置/速度 | `Query::EncoderEstimates` | `Response::EncoderEstimates { position, velocity }` |
| 读电机/编码器错误 | `Query::MotorError` / `Query::EncoderError` | `Response::MotorError(bits)` / `Response::EncoderError(bits)` |
| 读母线电压/电流 | `Query::VbusVoltage` / `Query::Iq` | `Response::VbusVoltage(volts)` / `Response::Iq { setpoint, measured }` |

例如准备清错：

```rust
use odrive_can_driver::{Driver, OperationId, PrepareError, protocol::Command};

fn prepare_clear_errors(driver: &mut Driver, now_ms: u64, deadline_ms: u64)
    -> Result<OperationId, PrepareError>
{
    driver.prepare_command(Command::ClearErrors, now_ms, deadline_ms)
}
```

取得 `id` 后，分别调用 `embedded_can::transmit_nb` / `transmit_blocking`、`embassy::transmit(...).await` 或 `SocketCan::send`。完整发送、接收、期限和错误分支见[三个后端示例](examples/README.md)。

**写命令的 `Submitted` 只表示后端接受了帧。** 取走该报告后仍应持续接收；需要确认 Idle 或清错效果时，比较后续 Heartbeat 的 `received_at_ms` 与报告的 `submitted_at_ms`，并检查目标状态/错误位。CANSimple 没有事务序号，Heartbeat 不是命令 ACK，查询的 `Observed` 也只表示在时间窗口内观察到同节点同类型反馈。

清错不会自动进入闭环；进入闭环不会自动设置安全目标；写零速度也不等于进入 Idle。这些步骤由应用根据设备模式、限值和保护策略显式安排。

## 异常处理

处理顺序是：**保留原始错误 → 查看 `report(id)` → 依据发送进展处理 → 满足条件后取走报告**。不要仅凭一个 I/O 错误判断“设备没收到”。

| 情况 | 含义与处理 |
|---|---|
| `PrepareError::Encode`、`DeadlineElapsed`、`ClockRollback` | 准备失败，没有此次发送。修正参数、绝对期限或时钟来源；不要跳过验证 |
| `PrepareError::Busy` | 上一操作或尚未取走的报告占用槽位。先处理它，不重建 Driver 来绕过未知状态 |
| TX `WouldBlock` | 明确未被后端接受，通常仍为 `Prepared`；继续接收和检查期限，稍后重试**同一 id** |
| RX 无帧 | embedded-can 返回 `Empty`，SocketCAN 返回 `WouldBlock`；不是设备故障，继续调度并 `tick` |
| `Failed` / `Cancelled` | 此操作已知未提交；取走报告后由应用决定是否发起新操作。`cancel` 仅适用于 `Prepared` |
| `TimedOut` | 看 `submitted_at_ms`：`None` 表示已知未提交，`Some` 表示查询已提交但反馈未及时观察到。取走报告，不据此认定设备未执行 |
| 发送 I/O 错误、发送 future 中途丢弃、接受后无法记录提交 | 可能是 `Unknown`。保留错误和报告，不自动重试；`take_report` 此时返回 `UnknownPending` |
| `Unknown` 的后续操作 | 先由应用确认底层不会再提交/迟到发送该帧，才调用 `acknowledge_unknown(id)`，随后 `take_report(id)`。这只释放追踪槽位，不证明设备未执行或已取消 |
| 接收错误、FD/RTR/扩展帧或无法解码的帧 | 处理后端错误与分类，并将原始帧交还总线分发方；接收失败不回滚已经提交的命令，也不自动清错或复位外设 |
| Heartbeat 过期、设备错误位非零 | 通信新鲜度与设备健康分开判断；保存完整错误位，再由应用决定停止、诊断、清错或恢复 |

blocking 接口没有可由本库保证的超时中断；嵌入式异步接口须由应用安排 timer/取消点。`tick` 只推进驱动记录，不会终止阻塞调用、撤销控制器队列或停止电机。SocketCAN 默认关闭错误通知，若需要总线错误帧，须通过 `bus.socket().set_error_filter(...)` 显式启用，见示例。

## 共享总线与支持范围

调用方统一读取、分发和提交发送。后端返回无关帧与可能被置换的 TX 帧；共享总线应用须继续处理它们。`embedded-can` trait 只表达 Classic/RTR，FD 和底层错误信息应在原生 HAL 层分发；Embassy 与 SocketCAN 保留原生帧形态。库不清空 RX、不重配总线，也不重建共享外设。

协议支持仅限 `odrive-can-protocol 0.1.2` 核对的 **ODrive `fw-v0.5.1`** 和 **MKS ODrive Mini `ODriveMINI-fw-v0.5.1-20250326`**。固定源码依据见[协议库支持矩阵](https://github.com/MRNIU/odrive-can-protocol#readme)，不能据此推断其他固件兼容。

完整 API 合同见 [rustdoc](https://docs.rs/odrive_can_driver)，接入代码和时序见 [examples](examples/README.md)，已完成验证及限制见[验证记录](docs/validation.md)，贡献与发布流程见 [CONTRIBUTING.md](CONTRIBUTING.md)。本项目采用 [MIT License](LICENSE)。
