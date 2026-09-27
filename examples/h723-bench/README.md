<!-- Copyright The odrive-can-driver Contributors -->
<!-- 职责：记录 H723 台架应用的连接、命令、证据等级与停止边界；不定义发布库 API 或产品安全策略。 -->

# STM32H723 ODrive CAN 台架

这是位于库包之外的 H723 `no_std` 应用，用于验证 Embassy 后端。它不把板级引脚、节点号或机械策略带入 `odrive_can_driver`。

`FDCAN2` 使用 `PB12`（RX）和 `PB13`（TX）、AF9，经 MCP2562FD 连接 ODrive；台架使用 Classic CANSimple、1 Mbit/s、axis0 node 1。`FDCAN1` 的 `PD0/PD1` 仅供同速率观察。25 MHz HSE 经 PLL2Q 提供 80 MHz FDCAN 内核时钟；USB OTG HS 的内部全速 PHY 使用 `PA11/PA12` 和 HSI48，提供 CDC 控制台。

## 运行边界

应用持续接收 FDCAN2 帧，并通过库的 Embassy 后端发送、接收和分类。`Submitted` 仅表示本地 FDCAN 队列接受帧，Heartbeat 不是写命令 ACK。`PHYSICAL` 表示收到 ODrive CAN 帧；`SOFTWARE_INJECTION`、`LOOPBACK`、`LOCAL`、`ABSENT_TARGET` 与 `RAW` 只表示各自的软件或控制器事件。

运行器在给定场景之后请求 H723 CAN Idle，并要求新的无错误 Idle Heartbeat。`fault` 和 `motion` 还经 Fibre 请求 Idle 并读一次状态；这份读回不能证明 H723 CAN Idle 已送达。任何 `STOP_UNKNOWN`、`RECOVERY_UNKNOWN`、`LOCAL_FAILURE`、`UNEXPECTED`、`NO_RESPONSE` 或 `NO_MOTION_EVIDENCE` 都是失败，且不会自动重试未知副作用。

下表为固件的 CDC 命令。主机运行器用 `motion+` / `motion-` 选择运动场景，转换为 `motion +` / `motion -`；`idle` 由运行器在结束时发送，不是可单独选择的场景参数。

| 命令 | 行为 |
| --- | --- |
| `status` | 对 node 1 发 Vbus RTR，输出实际回复的 IEEE-754 位值。 |
| `idle` / `recover` | 请求 Idle；`recover` 先 ClearErrors。两者均等待新的无错误 Idle Heartbeat。 |
| `fault` | 发送 Estop，观察 `0x4000` 错误位，再 ClearErrors 和 Idle。 |
| `absent` | 向 node 63 查询并等待超时；它不证明 Bus-Off 或帧已上线。 |
| `motion +` / `motion -` | 写零速度、进入闭环、发送 `±0.5 turn/s` 500 ms、再写零速度和 Idle；只以方向一致且不少于 `0.005 turn` 的位置变化证明运动。 |
| `ping`、`timer`、`raw`、`loopback`、`inject ...` | 仅软件、原始帧、控制器自检或内存注入诊断。 |

## 构建

本目录的 Cargo 配置锁定修复 RTR DLC 8 的 Embassy revision。构建外部 CAN 镜像：

```sh
cargo build --release
```

控制器内部 loopback 镜像：

```sh
cargo build --release --features internal-loopback
```

镜像仅链接 H723 前 768 KiB 应用区，保留最后 256 KiB 配置区。构建、刷写、USB 枚举、本地提交或软件注入本身不证明 ODrive 执行或运动。

## 主机运行

运行器需要 Python 3.9+、`pyserial` 和 `odrive==0.5.1.post0`。`--port` 指定 H723 CDC 设备；未提供时只在恰好发现一个 bench CDC 设备时自动选择。`--odrive-serial` 指定 Fibre ODrive USB 序列号；`fault` 和 `motion` 场景必须显式提供。`--baud` 仅是 CDC 终端设置，默认 `115200`。

```sh
python3 -m pip install pyserial odrive==0.5.1.post0
python3 host_runner.py --port <h723-cdc-port> --odrive-serial <odrive-usb-serial> \
  recover status fault absent inject-stale inject-deviceerror inject-noresponse
```

运动不会被隐式加入。操作者必须先在同一 Fibre 会话设置并读回适合现场的限制；运行器只读检查电流不超过 4 A、速度在 0.5 到 1 turn/s、watchdog 为 1 到 3 s、velocity/passthrough、Idle 且无错误、已校准并已 ready。它不修改这些配置，也不保存配置。运动期间不要另开 USB 轮询。

实际覆盖范围和限制见 [验证记录](../../docs/validation.md)。
