<!-- Copyright The odrive-can-driver Contributors -->
<!-- 记录本次台架的实际身份、配置、收发、运动与停止证据；不定义通用机械策略。 -->

# H723 / ODrive 实板验证记录

验证于 2026-09-26 至 2026-09-27。实际设备操作由连接设备的 `nzh-mbpm1`（macOS arm64）执行，固件在 Linux devbox 构建。没有把 macOS 当作 SocketCAN 主机。

## 身份、拓扑与构建

- H723 经 CMSIS-DAP `d8e91af5` 刷写；OpenOCD 识别 STM32H723 Cortex-M7，`program ... verify reset exit` 成功。镜像只链接 Flash 前 768 KiB，本次擦除不超过 `0x0801ffff`。
- ODrive USB serial `327834523034`，USB 自报硬件 3.6、固件 `0.0.0`、unreleased。用户确认这是 MKS ODrive Mini 和 `ODriveMINI-fw-v0.5.1-20250326`；未将自报版本冒充官方发布版。
- 主机 USB → H723 CDC 是命令与观察通路；主机 CANable2 → H723 FDCAN1 `PD0/PD1` 为被动观察通路。H723 **FDCAN2** `PB12/PB13` 经 MCP2562FD → ODrive，Classic CANSimple、1 Mbit/s、node 1。
- 主机 USB → ODrive Fibre 是独立配置和读回路径，不替代 H723 CAN 证据。CANable2 没有被宣称为 ODrive 直连。
- H723 使用 25 MHz HSE、PLL2Q 80 MHz CAN 内核时钟；USB OTG HS 内部 FS PHY 在 `PA11/PA12`，HSI48。CDC 枚举为 `/dev/cu.usbmodemh723_can_bench1`。

实际库依赖保持 crates.io `odrive-can-protocol 0.1.2`。Embassy 采用与参考应用相同的固定 Git revision [`7b08a9c7`](https://github.com/embassy-rs/embassy/tree/7b08a9c7d9a9fe620f4be25c4e7b86dd29f09f54)，通过工作区根 patch 统一来源；未依赖 EHA 私有 crate。台架源码和运行器在 [`examples/h723-bench`](../examples/h723-bench/README.md)。

## 配置与场景

初始预检：母线约 48.2 V，axis0/axis1 Idle、错误 0；Heartbeat 100 ms，watchdog 启用、2.5 s；axis0 velocity control、passthrough input；编码器已 ready，电机已 calibrated。原电流限制 64 A、速度限制 100 turn/s 未作为本次运动限值。

运动前通过 Fibre 显式配置并读回：电流限制 **4 A**、速度限制 **1 turn/s**、输入速度和力矩前馈为零；watchdog 仍启用 2.5 s，控制模式 2、输入模式 1。用户确认现场可进行运动。以下参数仅是此次受控台架选择，不是通用设备的机械安全保证。

| 场景 | 实际结果与层次 |
|---|---|
| Heartbeat | 本库 Embassy RX 收到 Classic `0x021`，8 B，例 `00 08 00 00 01 00 00 00`；保留完整 `axis_error=0x800`、state=Idle。`0x800` 是 watchdog 超时，不是 Estop。 |
| Vbus RTR 查询 | 固定 Git Embassy 发送 RTR DLC 8；收到 `0x037`，数据 `e0 91 40 42 00 00 00 00`，驱动为 Observed，电压 **48.142456 V**。 |
| 明确状态/清错 | `ClearErrors`、`SetAxisRequestedState(Idle)` 经库发送；观察到新的无错误 Idle Heartbeat。Heartbeat 是状态观察，不是命令 ACK。 |
| 设备错误与恢复 | CAN Estop 后观察到完整 `axis_error=0x4000`；显式 ClearErrors 与 Idle 后观察到无错误 Idle Heartbeat。故障注入层是设备 CANSimple Estop 命令，不是物理总线破坏。 |
| 无响应 | node 63 的 Vbus RTR 本地提交后到期，无匹配反馈。只证明该查询超时，不证明物理 Bus-Off 或远端未执行其他操作。 |
| 反馈过期、错误缓存、无响应注入 | H723 内存中构造的 stale/deviceerror/noresponse 场景通过，输出明确标为 SOFTWARE_INJECTION；不冒充物理链路掉线。 |
| 短时运动 | 在 Idle 先写零速度，读取位置，请求并观察 ClosedLoop；发送 **+0.5 turn/s、500 ms、零力矩前馈**，随后写零速度和 Idle。观察到新的无错误 Idle Heartbeat，并查询最终位置。 |
| 独立停止读回 | 最后再次显式发送 CAN Idle，并观察新的 Idle Heartbeat；Fibre 独立请求 Idle 后读回 `current_state=1`、`axis0.error=0`。 |

运动位置原始 IEEE-754 位值为：起点 `0xc0cebe99` = **-6.460766315 turn**；终点 `0xc0cdf6ad` = **-6.436361790 turn**；差值 `0x3cc7ec00` = **+0.024404526 turn**。它与指令同号并超过事先设定的 0.005 turn 判据。这里只证明发生了运动，不声称目标速度已达到或位置跟踪精度已验证。此前 `±0.1 turn/s、500 ms` 的位移均低于判据，已记录为未通过运动证明，没有降低阈值制造成功。

## 定位出的实际问题

- CDC 全速包上限为 64 B。长日志原先写失败，分包后 timer 与结果日志正常；先前静默不能被认定为时基故障。
- 台架必须持续服务 CAN RX，不能只在收到 USB 命令后接收；程序现于等待命令时更新反馈，并在快速查询前通过正常库接收路径处理有限积压帧。没有复位或清空共享外设。
- registry `embassy-stm32 0.6.0` 会清除 DLC 大于零的发送 RTR 位，导致查询成为数据帧。固定 Git revision 修复该问题；相同台架使用修复后版本取得了真实 RTR 回复。发布库的消费方必须采用 README 中的工作区根 patch，Cargo 不会向下游传递该配置。
- 主机旧版 Fibre 曾在成功读回 Idle 后因后台 USB 线程与 Python 退出清理竞争而发生 native SIGTRAP，相关运行的进程状态没有记为通过。运行器改为显式取消并等待 Fibre 线程后，非运动 `recover fault` 序列及独立 preflight 均正常返回 0；CAN 与 Fibre 仍读回 Idle、错误 0。已获得的运动帧与停止读回保留为设备证据，未将旧脚本异常隐藏为成功。
- 本次内部 loopback 只证明控制器与库收发；原始帧、查询回复、故障位和运动位置才是外部设备证据。诊断期错标寄存器地址所得结论没有用于验收。

## 最终状态与未验证项

最后停止观察为 **Idle，axis error 0**，CAN 与 Fibre 均已确认。watchdog 保持启用；测试结束后库不发送后台保活，后续无寻址流量时可能再次出现 `0x800`，它不代表自动恢复运动。4 A 和 1 turn/s 是本次运行期配置，未执行保存或恢复原配置；设备重启后不得假设这些限值仍在。

没有验证物理 Bus-Off、电源故障或线缆断开。embedded-can 验证使用兼容 trait 的受控软件实现；SocketCAN 使用真实 Linux vcan 套接字，没有桥接到本台架。不能声称三个后端均完成实板验证，也不能从本次单方向运动推断全行程、负载或机械保护性能。
