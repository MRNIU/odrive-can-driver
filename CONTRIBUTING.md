<!-- Copyright The odrive-can-driver Contributors -->
<!-- 本文件说明驱动库的贡献边界、验证层次和发布要求。 -->

# 贡献指南

欢迎提交 [Pull Request](https://github.com/MRNIU/odrive-can-driver/pulls)。问题应给出最小复现、crate 与 Rust 版本；CAN 问题还应给出固件版本、节点 ID、帧类型、CAN ID、DLC、有效载荷和观察时间。

请遵守 [README](README.md) 的支持范围、后端要求和操作语义。驱动只记录本地 I/O 进展与协议观察；共享总线调度、外设配置、机械限值、保护许可和设备恢复仍由应用负责。不要复制 `odrive-can-protocol` 的编解码实现，也不要把新增后端表述为新增固件支持。

公开 API 需有中文 rustdoc，说明时间单位、watchdog 影响和结果语义。每个源码文件保留版权和职责说明；不得删除 MIT 许可或适用的原版权。

## 验证

PR 说明触发条件、变更后行为、兼容性影响、风险边界及实际运行的检查。操作语义变化还须同步 README、示例、rustdoc 和行为测试。按变更范围运行 [examples 指南](examples/README.md) 中相应后端的检查；Linux vcan 是软件集成，实板结果与未覆盖项目见 [验证记录](docs/validation.md)。

默认核心的检查入口如下；后端示例需启用对应 feature，交叉目标由应用选择。

```sh
cargo fmt --check
cargo test --locked
cargo clippy --locked --all-targets -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --locked --no-deps
```

## 发布

发布前完成三后端的适用软件检查、H723 主路径实板验证、`cargo package --locked` 和 `cargo publish --locked --dry-run`。确认 crates.io、docs.rs 与仓库链接后，再创建 tag 和 GitHub Release。

提交使用中文 Conventional Commits、精确暂存和 `git commit --signoff`；AI 协作添加 `Co-authored-by: OpenAI Codex <codex@openai.com>`。不要提交凭据、现场记录或无关构建产物。
