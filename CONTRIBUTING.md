<!-- Copyright The odrive-can-driver Contributors -->
<!-- 本文件说明驱动库的贡献边界、验证层次和发布要求。 -->

# 贡献指南

欢迎提交 [Pull Request](https://github.com/MRNIU/odrive-can-driver/pulls)。请给出可复现的最小代码、crate 与 Rust 版本；与 CAN 有关的问题应同时给出固件 revision、节点 ID、帧形态、CAN ID、DLC、有效载荷和观察时间。

## 维护边界

- 这是单个可发布 library crate；默认构建保持 Rust 2024、`no_std`、无 `alloc`，并仅依赖 `odrive-can-protocol 0.1.2`，核心 MSRV 为 Rust 1.85。
- 共享状态机只记录协议操作和本地 I/O 进展。CAN 外设、任务、时钟源、位速率、机械限值、保护许可及设备恢复属于应用。
- 使用 crates.io 的 `odrive-can-protocol 0.1.2` 完成 CANSimple 编解码和帧适配；不得复制协议实现或扩大固件支持范围。
- 三个 optional 后端封装真实 I/O，但调用方仍拥有共享总线的 RX/TX 调度；不得清空 RX、重配或重建外设，也不得吞掉其他节点帧。
- `Submitted` 是本地提交，`Observed` 是时效范围内观察到的查询反馈，Heartbeat 不是写命令 ACK。取消 future 不证明未发送；未知副作用不能自动重试。
- 不自动清错、进入闭环、恢复目标或决定产品许可。ODrive `fw-v0.5.1` 的寻址帧（含 RTR）会喂 watchdog，库不得因此隐式 query 或保活。
- rustdoc 与文档使用中文，标识符使用英文。公开 API 必须说明前提、时间单位、watchdog 影响、发送进展与结果语义。
- 每个文件保留 `Copyright The odrive-can-driver Contributors` 和职责说明；MIT 许可与适用原版权不得删除。

## 后端与兼容性

| feature | 维护要求 |
|---|---|
| 默认 | 在 `no_std` 目标上保持无分配核心；MSRV Rust 1.85。 |
| `embedded-can` | 通过 `embedded-can 0.4` 的 blocking 与 `nb` 真实收发，区分明确未接收和发送副作用未知。 |
| `embassy-stm32` | 使用名义 `embassy-stm32 0.6.0` 原生异步接口；芯片 feature 由消费者选择，首版目标为 STM32H723 FDCAN，完整 RTR 组合必须使用 README 所列固定 Git revision patch，当前最低已验证 Rust 1.98.1。 |
| `socketcan` | 仅 Linux、`socketcan 4`、无运行时；MSRV Rust 1.89。 |

新增后端不等于新增 ODrive 固件支持。任何新固件版本都必须先在 `odrive-can-protocol` 以固定 revision 建立编码合同，再由本库更新依赖与支持矩阵。

消费方必须在其工作区根应用 README 的六包 `[patch.crates-io]` 配置；Cargo 发布不会传递该 patch。crates.io 的 `embassy-stm32 0.6.0` 在 FDCAN 中只会对 DLC 0 设置 RTR 位，不能构成 CANSimple DLC 8 RTR 查询的完整验证组合。固定 Git revision `7b08a9c7d9a9fe620f4be25c4e7b86dd29f09f54` 保留 `header.rtr()`。

固定 Git Embassy H723 组合当前以 Rust 1.98.1 验证，未进行真实最小版本二分。Rust 1.89 的 target check 会在传递依赖 `xarxa-driver` 的 `cfg_select!` 处失败；不要将默认/`embedded-can` 的 Rust 1.85 或 SocketCAN 的 Rust 1.89 外推到 Embassy。

## 本地验证

以下命令分层证明软件结果；只有改动涉及的层次才需要运行，且成功结果无需对未变化输入重复检查。

```sh
cargo fmt --check
cargo test --locked --all-targets
cargo test --locked --doc
cargo clippy --locked --all-targets -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --locked --no-deps
cargo build --locked --lib --target thumbv7em-none-eabihf
cargo build --locked --lib --target thumbv6m-none-eabi
cargo +1.85.0 check --locked --lib
cargo +1.85.0 check --locked --lib --features embedded-can
```

可选后端还须在适用目标上验证：

```sh
cargo test --locked --all-targets --features embedded-can
cargo clippy --locked --all-targets --features embedded-can -- -D warnings
cargo check --locked --lib --target thumbv7em-none-eabihf --features embedded-can

cargo +1.89.0 test --locked --all-targets --features socketcan
cargo +1.89.0 clippy --locked --all-targets --features socketcan -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo +1.89.0 doc --locked --no-deps --features socketcan

cargo +1.98.1 check --locked --lib --target thumbv7em-none-eabihf \
  --features embedded-can,embassy-stm32,embassy-stm32/stm32h723vg
cargo +1.98.1 clippy --locked --lib --target thumbv7em-none-eabihf \
  --features embedded-can,embassy-stm32,embassy-stm32/stm32h723vg -- -D warnings
```

Linux vcan 集成测试须显式运行；它不会在常规 `cargo test` 中自动执行：

```sh
cargo test --locked --features socketcan --test socketcan -- --ignored
```

该测试需先创建独立 vcan 接口，详细命令见 [examples/README.md](examples/README.md)。它证明 Linux 套接字路径，不得表述为物理 Bus-Off 或实板动作验证。Embassy H723 的构建与 RTR 实板验证都必须在应用了固定 Git patch 的工作区执行；不得把 registry `0.6.0` 或 docs.rs API 构建视为总线行为证据。

## PR 与发布

PR 应说明触发条件、变更后的行为、兼容性影响、风险边界及实际运行过的验证命令。操作语义变更必须同时更新 rustdoc、README、示例和相应测试。

提交使用中文 Conventional Commits，精确暂存，并以 `git commit --signoff` 添加 DCO。AI 协作添加 `Co-authored-by: OpenAI Codex <codex@openai.com>`。不要提交凭据、本地设备记录或无关构建产物。

发布前必须完成默认、`embedded-can`、SocketCAN 和应用固定 Git patch 的 Embassy H723 的适用软件验证，以及 H723 主路径的实际设备验证；同时运行 `cargo package --locked` 与 `cargo publish --locked --dry-run`。维护者仅在已配置 crates.io 凭据的环境发布；确认注册表、docs.rs 和仓库链接后，再创建 tag 与 GitHub Release。软件检查和发布完成均不替代实板资格验证。
