<!-- Copyright The odrive-can-driver Contributors -->
<!-- 本文件约定驱动库的技术边界。 -->

# 技术边界

- 单个可发布 library crate，package 与导入名为 `odrive_can_driver`；Rust stable、2024 Edition，`default = []`，共享核心保持 `no_std`、无动态分配。
- `protocol` 是内置的硬件无关编解码层，按固件版本划分模块；它不访问外设、不维护时序，也不决定控制策略。新增协议支持须核对固定版本源码，并更新 README 支持矩阵和依据。
- 三个可选后端封装真实 I/O，共用 `Driver` 操作状态机。共享总线的读取、分发与发送所有权属于调用方；不清空 RX、不重配或重建共享外设。
- 区分未调用、本地提交、设备反馈、失败、超时和未知。Heartbeat 不是写命令 ACK；取消 future 不证明未发送。未知副作用不得自动重试。
- 不自动清错、闭环、恢复目标或决定产品许可。应用配置芯片、时钟、引脚、位速率、机械限值和保护策略。
- 未知协议状态与错误位是有效数据，不得截断或转为解码失败。rustdoc 说明 API 前提、时间单位、watchdog 影响和结果语义；改编代码保留适用版权，MIT LICENSE 不得删除。
