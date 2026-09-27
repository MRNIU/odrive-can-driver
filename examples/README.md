<!-- Copyright The odrive-can-driver Contributors -->
<!-- 本文件说明示例应用的责任、后端接入前提和验证证据边界。 -->

# 示例与后端接入

本目录中的示例是独立消费方应用，不属于发布库的默认 API 表面。它们负责选择芯片、配置 CAN 外设和总线参数、建立单调时钟、设置产品许可，并显式处理设备动作的安全边界。

所有示例都遵循同一操作顺序：准备命令或查询，取得 `SendAttempt` 后提交 `attempt.frame()`，根据底层结果记录 `submitted`、`would_block` 或 `not_sent`，在统一 RX 分发处调用 `ingest`，以 `tick` 推进期限，并依据 `OperationReport` 判断本地提交、观察反馈、已知未提交、超时或未知结果。每个终态报告都须以 `take_report` 取走后，才能准备下一操作。写命令的 `Submitted` 不是设备 ACK，查询的 `Observed` 也不携带 CANSimple 事务身份。

[`shared_bus.rs`](shared_bus.rs) 是默认 feature 的最小可运行边界示例：

```sh
cargo run --example shared_bus
```

## `embedded-can`

```toml
[dependencies]
odrive_can_driver = { version = "0.1.0", features = ["embedded-can"] }
```

应用以其已有的 `embedded-can 0.4` 外设实现调用 blocking 或 `nb` 发送。对 `nb::Error::WouldBlock`，仅在后端明确尚未接收帧时调用 `would_block` 并稍后重试同一个操作；`not_sent` 会结束为 `Failed`。任何原生 I/O 错误或无法证明未发送的退出路径都保留为 `Unknown`。共享总线的 RX 由应用先分发，不能把其他节点帧交给本驱动后丢弃。

## Embassy STM32H723

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

这段 `[patch.crates-io]` 必须位于消费方工作区根；Cargo 发布不会把本库的 patch 传递到你的工作区。crates.io `embassy-stm32 0.6.0` 在 FDCAN RTR 发送中只为 DLC 0 设置 RTR 位，CANSimple 的 DLC 8 查询会变成数据帧而没有 ODrive 回复。固定 revision 使用 `header.rtr()` 保留 RTR 位，才是完整查询组合。

Embassy 示例使用原生异步 FDCAN，但应用仍独占其外设初始化、时钟、引脚、位速率、任务和共享 RX 分发。H723 是当前示例目标，不代表其他 STM32 或 ODrive 固件版本已经验证。固定 Git H723 组合当前以 Rust 1.98.1 验证；Rust 1.89 会在传递 `xarxa-driver` 的 `cfg_select!` 处失败，且本次未为寻找真实 MSRV 做版本二分。编译此组合：

```sh
cargo +1.98.1 check --locked --target thumbv7em-none-eabihf \
  --features embedded-can,embassy-stm32,embassy-stm32/stm32h723vg
```

已知 H723 台架的独立、非发布应用位于 [h723-bench](h723-bench/README.md)。它记录板级 FDCAN、USB 观察和刷写门禁；它的构建或刷写不构成 CAN 通信、设备执行或机械动作的证据。

docs.rs 只展示 library API，既不应用消费方工作区的 patch，也不验证 RTR 或任何实际 CAN 总线行为。

## Linux SocketCAN 与 vcan

```toml
[dependencies]
odrive_can_driver = { version = "0.1.0", features = ["socketcan"] }
```

SocketCAN 后端使用非阻塞 `socketcan 4`，没有 Tokio。示例只在 Linux 运行，要求 Rust 1.89。创建一次性 vcan 后显式运行集成测试：

```sh
sudo ip link add dev odrive-vcan type vcan
sudo ip link set odrive-vcan up
cargo test --locked --features socketcan --test socketcan -- --ignored
sudo ip link delete odrive-vcan
```

该命令需要创建网络设备的权限。vcan 测试只验证 Linux 套接字的发送、接收和驱动状态路径；它不构成实板 CAN、设备配置、watchdog、Bus-Off 或机械运动的证据。

真实接口上可运行一次无副作用的母线电压 RTR 查询：

```sh
cargo +1.89.0 run --locked --features socketcan --example socketcan_query -- can0 1
```

参数依次为已配置的 SocketCAN 接口名和 ODrive 节点号。示例只发送这一次 `VbusVoltage`
查询，输出观察到的电压、本地超时或副作用未知；它不建立后台保活、不会发送控制命令，也不
把 Heartbeat 当作查询结果。只有观察到 `VbusVoltage` 时它以零退出码结束；超时、未知或
本地失败会输出明确消息并以非零退出。该示例独占其 SocketCAN 描述符的接收循环；共享总线
应用应在自己的统一接收循环中分发原始帧，而不要与本示例并发消费同一队列。

## 安全与 watchdog

ODrive `fw-v0.5.1` 对每一寻址 CANSimple 帧（含 RTR）喂 watchdog。因此示例不在后台发送 query、Heartbeat 或保活帧，也不自动清错、进入闭环、恢复目标或重试未知副作用。应用必须显式决定这些动作，并为超时、`Unknown` 和设备错误保留自己的 fail-safe 策略。
