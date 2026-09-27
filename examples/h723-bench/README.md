<!-- Copyright The odrive-can-driver Contributors -->
<!-- 职责：记录 H723 台架应用的连接、命令、证据等级与停止边界；不定义发布库 API 或产品安全策略。 -->

# STM32H723 ODrive CAN 台架

这是面向已知 H723 板的独立、未发布 `no_std` 应用，位于库包之外。它不把板级引脚、USB、节点号、速度、电流或机械策略带入可发布的 `odrive_can_driver`。

`FDCAN2` 使用 `PB12`（RX）和 `PB13`（TX）、AF9，经板载 MCP2562FD 连接 ODrive；本台架采用 Classic CANSimple、1 Mbit/s、axis0 node 1。`FDCAN1` 的 `PD0/PD1` 是同速率的被动主机观察路径。25 MHz HSE 经 PLL2Q 提供 80 MHz FDCAN 内核时钟。USB OTG HS 的内部全速 PHY 使用 `PA11/PA12` 和 HSI48，提供 CDC 控制台；RTT 只作辅助，板上没有 SWO/ITM 路由。

## 证据边界

应用持续接收 FDCAN2 帧，并通过库的 Embassy 后端提交、接收和分类。`Submitted` 仅表示本地 FDCAN 队列接受帧；Heartbeat 也不是某条写命令的 ACK。`PHYSICAL` 输出必须包含实际收到的 ODrive CAN 帧。`SOFTWARE_INJECTION`、`LOOPBACK`、`LOCAL`、`ABSENT_TARGET` 与 `RAW` 都各有较窄含义，不能替代设备执行、闭环或机械运动证据。

CDC 全速 bulk 包最多 64 B，固件会分包输出。主机运行器先等待 boot banner；除控制器自检和软件注入外，所有场景都在 `finally` 中发起一次新的 H723 CAN `Idle`，并要求新的 error-free Idle Heartbeat。motion 与 fault 还通过固定 ODrive 序列号作一次 Fibre Idle 写入和一次 `current_state`/`axis0.error` 边缘读回；runner 仅接受 `current_state == 1` 且 `axis0.error == 0`。此 Fibre fallback 不能证明 H723 的 CAN Idle 已到达 ODrive。

`host_runner.py` 可在一个 CDC 会话中接收一个或多个场景，例如 `recover status fault absent inject-stale`。它按给定顺序运行，首个非终态即停止后续场景，最后只发送一次 H723 CAN `Idle`；序列含 `fault` 或 `motion` 时再作一次 Fibre 边缘读回。motion 不会被隐式加入，必须显式传入。每个 motion 场景前，runner 以 Fibre **只读**检查 `current_lim` 在 `(0, 4] A`、`vel_limit` 在 `[0.5, 1] turn/s`、watchdog 已启用且 timeout 在 `[1, 3] s`、velocity/passthrough 的 `control_mode == 2`/`input_mode == 1`、Idle/error-free、motor 已校准和 encoder ready；它绝不自动修改这些配置。

## CDC 命令

| 命令 | 范围与成功证据 |
| --- | --- |
| `ping` / `timer` | 仅 CDC 与 Embassy timer 软件诊断。 |
| `raw` | 最多读取三帧 FDCAN2 原始帧；`RAW_TIMEOUT` 只是没有更多可读帧，`RAW_DONE` 才是本次有界读取完成。 |
| `loopback` | 仅 `--features internal-loopback` 镜像可用，证明控制器/库 TX-RX 路径；不会驱动外部 CAN 引脚或 ODrive。 |
| `status` | 向 node 1 发送 Vbus RTR；成功输出实际 IEEE-754 位值。 |
| `idle` | 提交 CAN Idle 后，要求新的 `axis_error == 0`、Idle Heartbeat。 |
| `fault` | 提交 Estop 后，要求新的 Heartbeat 含 v0.5.1 `ESTOP_REQUESTED` 位 `0x4000`；随后显式 `ClearErrors`、Idle，并要求 error-free Idle Heartbeat。`0x800` 是 watchdog 错误，不能作为 Estop 证据。 |
| `absent` | 对 node 63 发送 Vbus RTR。`ABSENT_TARGET` 只表示本地提交后无匹配回复，既不证明物理 Bus-Off，也不证明帧已在线上传输。 |
| `motion +` / `motion -` | 在仍为 Idle 时先写零速度以清除旧目标，再读 EncoderEstimates、请求 ClosedLoop、发送 `±0.5 turn/s` 500 ms、写零速度和 Idle。只有 error-free Idle Heartbeat 后的 EncoderEstimates 位移与命令同号且绝对值不少于 `0.005 turn`，才报告运动证据。它不证明机械行程或坐标安全。 |
| `recover` | 显式 `ClearErrors`、Idle 和新的 error-free Idle Heartbeat。 |
| `inject stale` / `inject deviceerror` / `inject noresponse` | 仅在内存中注入 driver 场景，不访问物理 CAN。 |

`STOP_UNKNOWN`、`RECOVERY_UNKNOWN`、`LOCAL_FAILURE`、`UNEXPECTED`、`NO_RESPONSE` 和 `NO_MOTION_EVIDENCE` 都不是通过结果。发送在期限内被取消或无法确认时保持 `Unknown`，不会自动重试。

## 运动配置前提

台架运动使用明确授权的 RAM 配置，未调用 `save_configuration()`，不把本次临时设置写入 NV。下列 Python 片段必须在 motion 前由操作者显式执行，并在同一 Fibre 会话读回；runner 只读检查，绝不替操作者执行这一段。

```python
from host_runner import odrive_session

with odrive_session("327834523034") as odrv:
    axis = odrv.axis0
    axis.motor.config.current_lim = 4.0
    axis.controller.config.vel_limit = 1.0
    axis.config.enable_watchdog = True
    axis.config.watchdog_timeout = 2.5
    axis.controller.config.control_mode = 2  # VELOCITY_CONTROL
    axis.controller.config.input_mode = 1    # PASSTHROUGH
    axis.controller.input_vel = 0.0
    axis.watchdog_feed()
    axis.clear_errors()
    axis.requested_state = 1                 # IDLE

    print(
        axis.motor.config.current_lim,
        axis.controller.config.vel_limit,
        axis.config.enable_watchdog,
        axis.config.watchdog_timeout,
        axis.controller.config.control_mode,
        axis.controller.config.input_mode,
        axis.current_state,
        axis.error,
        axis.motor.is_calibrated,
        axis.encoder.is_ready,
    )
```

读回必须分别为 `4.0`、`1.0`、`True`、`2.5`、`2`、`1`、Idle `1`、error `0`、motor calibrated `True` 和 encoder ready `True`，才可执行显式 motion 场景。

## 实板记录绑定

本台架使用与参考应用相同的 Embassy revision `7b08a9c7d9a9fe620f4be25c4e7b86dd29f09f54`，由本目录 Cargo.toml 的固定 Git patch 与 Cargo.lock 锁定。它修复 registry `embassy-stm32 0.6.0` 的 RTR DLC 8 发送缺陷；该 revision 属于 Embassy 仓库，不是 EHA 固件版本。本台架不依赖 EHA 私有 crate。实际场景、数值与验证边界见 [实板记录](../../docs/hardware-validation.md)。

## 构建

在本目录中构建正常外部 CAN 镜像：

```sh
cargo build --release
```

控制器内部 loopback 镜像：

```sh
cargo build --release --features internal-loopback
```

镜像只链接 H723 前 768 KiB 应用区，保留最后 256 KiB 配置区。刷写必须使用另行记录的 probe serial、目标身份、ELF 和停止条件。构建、刷写、USB 枚举、本地提交或软件注入均不证明 CAN 通信、ODrive 执行或物理运动。

## 主机运行

在实际连接 USB 设备的主机安装 `pyserial` 和 `odrive==0.5.1.post0`，使用 Python 3.9 或更高版本。下面的默认序列不包含运动；运动需完成上述配置后显式选择 `recover motion+` 或 `recover motion-`。

```sh
python3 -m venv /tmp/odrive-can-bench
/tmp/odrive-can-bench/bin/python -m pip install pyserial odrive==0.5.1.post0
/tmp/odrive-can-bench/bin/python host_runner.py --port /dev/cu.usbmodemh723_can_bench1 recover status fault absent inject-stale inject-deviceerror inject-noresponse
```

旧版 Fibre 会启动后台 discovery 和 receiver 线程。运行器通过 `odrive_session` 的取消 token 显式终止并等待这些线程，避免 Python 退出时与 libusb 清理竞争；清理超时同样返回失败。上面的配置片段也复用该上下文管理器，应在 `host_runner.py` 所在目录运行。不要在运动期间另开 USB 轮询进程。
