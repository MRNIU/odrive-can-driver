<!-- Copyright The odrive-can-driver Contributors -->
<!-- 本文件约定驱动库的技术边界。 -->

# 技术边界

- 单个可发布 library crate，package 与导入名为 `odrive_can_driver`；Rust stable、2024 Edition，`default = []`，共享核心保持 `no_std`、无动态分配。
- `protocol` 是内置的硬件无关编解码层，按固件版本划分模块；它不访问外设、不维护时序，也不决定控制策略。新增协议支持须核对固定版本源码，并更新 README 支持矩阵和依据。
- Rust 的三个可选后端封装真实 I/O，共用 `Driver` 操作状态机。共享总线的读取、分发与发送所有权属于调用方；不清空 RX、不重配或重建共享外设。
- 原生 C17 静态库独立通过 CMake 构建，编解码、每节点原始反馈缓存和单槽 Driver 的可观察行为与 Rust 共用测试向量。调用方提供固定存储、单调微秒时钟和 I/O。C Driver 须在运行时验证实例身份、发送凭证、每次尝试与候选反馈。
- 区分未调用、本地提交、设备反馈、失败、超时和未知。Heartbeat 不是写命令 ACK；取消 future 不证明未发送。未知副作用不得自动重试。
- 不自动清错、闭环、恢复目标或决定产品许可。应用配置芯片、时钟、引脚、位速率、机械限值和保护策略。
- 未知协议状态与错误位是有效数据，不得截断或转为解码失败。Rust 使用 rustdoc，C 使用 Doxygen，公开接口说明前提、时间单位、watchdog 影响和结果语义。改编代码保留适用版权，MIT LICENSE 不得删除。
