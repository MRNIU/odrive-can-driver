<!-- Copyright The odrive-can-driver Contributors -->
<!-- 本文件说明贡献者需要遵守的公共接口和兼容性边界。 -->

# 贡献指南

欢迎提交 [Pull Request](https://github.com/MRNIU/odrive-can-driver/pulls)。问题应给出最小复现、crate 与 Rust 版本；CAN 问题还应给出固件版本、节点 ID、帧类型、CAN ID、DLC、有效载荷和观察时间。

请遵守 [README](README.md) 的支持范围、后端要求和操作语义。驱动只记录本地 I/O 进展与协议观察；共享总线调度、外设配置、机械限值、保护许可和设备恢复仍由应用负责。不要复制 `odrive-can-protocol` 的编解码实现，也不要把新增后端表述为新增固件支持。

公开 API 的 rustdoc 应说明时间单位、watchdog 影响和结果语义。保留 MIT 许可及适用的原版权。

## 验证

PR 说明触发条件、变更后行为、兼容性影响和风险边界。操作语义变化须同步 README、示例、rustdoc 和行为测试；按变更范围运行相应检查。
