<!-- Copyright The odrive-can-driver Contributors -->
<!-- 记录 0.1.0 的软件与实板验证层次；不定义 API 或机械策略。 -->

# 0.1.0 验证记录

## 软件

共享核心和嵌入式目标保持 `no_std`、无动态分配。以下结果覆盖实际行为，而非协议字节编解码矩阵。

| 范围 | 结果 |
|---|---|
| 共享核心 | 16 个行为测试和 1 个 rustdoc 示例通过，覆盖提交、观察、超时、未知结果与无关帧。 |
| `embedded-can` | 7 个受控收发测试通过，覆盖 blocking、`nb`、原始错误、替换帧和期限。 |
| `embassy-stm32` | 3 个 H723 feature 下的原生帧行为测试通过；异步总线 I/O 另见下方实板。 |
| `socketcan` | 3 个受控测试与 1 个 Linux vcan 集成测试通过；vcan 是软件集成，不是实物总线。 |
| 质量与目标 | rustfmt、相应 feature 的 Clippy 与 `-D warnings` rustdoc 通过；默认 `thumbv6m-none-eabi`、`thumbv7em-none-eabihf` 以及 H723 Embassy 组合构建通过。 |
| Rust 版本 | 默认与 `embedded-can` 以 Rust 1.85 验证，SocketCAN 以 Rust 1.89 验证；固定 Git Embassy H723 组合以 Rust 1.98.1 验证，未对其 MSRV 二分。 |
| 发布 | package、publish dry-run 和 [CI](https://github.com/MRNIU/odrive-can-driver/actions/runs/36287582597) 通过；[0.1.0 已发布](https://crates.io/crates/odrive_can_driver/0.1.0)，[API 文档](https://docs.rs/odrive_can_driver/0.1.0/odrive_can_driver/) 已生成。 |

H723 的 RTR 查询使用 Embassy 固定 revision `7b08a9c7d9a9fe620f4be25c4e7b86dd29f09f54`。它保留 DLC 8 RTR 位；registry `embassy-stm32 0.6.0` 不具备这一行为。消费方的 patch 与版本要求见 [examples 指南](../examples/README.md#embassy-stm32)。

## 实板

验证设备为 MKS ODrive Mini，固件来源确认为 `ODriveMINI-fw-v0.5.1-20250326`（USB 自报 `0.0.0 unreleased`）。H723 通过 FDCAN2 连接 ODrive，采用 Classic CANSimple、1 Mbit/s、axis0 node 1。FDCAN2 使用 `PB12`（RX）和 `PB13`（TX），经 MCP2562FD；25 MHz HSE 经 PLL2Q 提供 80 MHz CAN 内核时钟。USB CDC 仅作控制与观察，ODrive Fibre USB 仅作独立读回。

运动前显式读回运行期限制：4 A、1 turn/s、watchdog 2.5 s、velocity/passthrough，轴已校准且编码器 ready。它们不是通用机械安全参数，也没有持久化。

| 场景 | 结果 |
|---|---|
| Heartbeat 与查询 | 收到 Classic Heartbeat；DLC 8 RTR Vbus 查询得到 `48.142456 V` 的匹配回复。 |
| 指令与恢复 | `ClearErrors`、Idle 和 Estop 均通过库提交；Estop 后观察到 `axis_error=0x4000`，随后 ClearErrors 和 Idle 得到无错误 Idle Heartbeat。Heartbeat 只证明状态观察，不是写命令 ACK。 |
| 无响应与注入 | 对不存在节点的查询本地提交后超时。陈旧反馈、设备错误和无响应注入均只在软件内存中执行，未表述为物理断线或 Bus-Off。 |
| 运动与停止 | 以 `+0.5 turn/s` 运行 500 ms，位置从 `-6.460766315` 变为 `-6.436361790 turn`，差值 `+0.024404526 turn`。随后写零速度和 Idle，观察到新的无错误 Idle Heartbeat；独立读回也为 Idle、错误 0。该结果只证明短时运动，不证明速度跟踪、行程、负载或机械保护。 |

## 限制

`embedded-can` 仅以兼容 trait 实现验证；SocketCAN 只经 Linux vcan 验证，没有桥接到这套设备。未验证物理 Bus-Off、电源故障或线缆断开。watchdog 保持启用，测试结束后没有后台保活；后续没有寻址流量时可能出现 `0x800`，不能据此推断自动恢复运动。

`Submitted`、单元测试、交叉编译、USB 枚举或刷写成功都不等同于设备执行。完整操作状态和异常处理以 README、示例与 rustdoc 为准。
