<!-- Copyright The odrive-can-driver Contributors -->
<!-- 记录 0.1.0 的软件验证范围和证据边界；实际设备记录单独维护。 -->

# 0.1.0 验证记录

## 软件

在 Linux ARM64 上完成以下验证。共享核心和两个嵌入式构建目标不依赖 `std` 或 `alloc`；默认依赖树只有 `odrive-can-protocol 0.1.2`。

| 范围 | 结果 |
|---|---|
| 共享核心 | 16 个行为测试通过；1 个 rustdoc 示例通过 |
| embedded-can | 7 个受控收发测试通过，覆盖 nb、blocking、原始错误、替换帧、跨期限、扩展帧保留 |
| Embassy | 3 个 H723 feature 下的原生帧行为测试通过；实际异步 I/O 另见实板记录 |
| SocketCAN | 3 个受控测试通过；1 个显式 vcan 集成测试通过，实际使用本库与 SocketCAN 4 |
| 质量 | rustfmt；默认及 embedded-can + socketcan 的 all-targets Clippy；Rust 1.98.1 下 H723 `embedded-can + Embassy` 组合 Clippy，均以警告为错误 |
| rustdoc | 默认及 `embedded-can`、SocketCAN、固定 Git Embassy H723 三后端组合，`-D warnings` 通过；docs.rs 只展示 API，不证明实际总线行为 |
| 默认无 std | `thumbv7em-none-eabihf`、`thumbv6m-none-eabi` 构建通过 |
| H723 | 固定 Git Embassy 下 `embedded-can,embassy-stm32,embassy-stm32/stm32h723vg` 实际目标构建与 Clippy 通过 |
| Rust 1.85 | 默认和 embedded-can 检查通过 |
| Rust 1.89 | SocketCAN 验证通过；固定 Git Embassy H723 的 target check 在传递依赖 `xarxa-driver` 的 `cfg_select!` 处失败 |
| Rust 1.98.1 | 固定 Git Embassy H723 组合验证通过；未为寻找真实 MSRV 做版本二分 |
| 发布包 | `cargo publish --locked --dry-run --allow-dirty` 通过；包含 19 个文件，含 library、测试和两个库示例，自动排除嵌套 bench |

Embassy 使用与 EHA lock 一致的 Git revision `7b08a9c7d9a9fe620f4be25c4e7b86dd29f09f54`，名义 package 版本为 `embassy-stm32 0.6.0`。该 revision 修复 FDCAN RTR 的 DLC 保留：registry `0.6.0` 仅在 DLC 0 设置 RTR 位，CANSimple 的 DLC 8 RTR 查询会成为数据帧；固定 revision 使用 `header.rtr()` 保留 RTR 位。默认及 embedded-can 的最低支持版本仍为 Rust 1.85，SocketCAN 的最低验证版本为 Rust 1.89；不得将这些结论外推为固定 Git Embassy 的 MSRV。

## 独立审查

独立核心与后端审查后修复了：确定未发送与未知结果的混淆、未取报告被覆盖、同毫秒查询观察、发送后时钟采样、置换帧在迟到提交时丢失，以及扩展 ID 解码失败时原始帧丢失。对应行为有回归覆盖。共享核心、后端、测试、README 与 rustdoc 的范围内无未解决的 P1/P2。

## 证据边界

受控 embedded-can 测试使用兼容 trait 实现；它证明实际 trait 调用和错误传播，未连接物理 CAN 适配器。vcan 在真实 Linux 内核和套接字上执行，属于软件集成，未桥接至实物。测试中的原生错误帧和超时注入不证明物理 Bus-Off。

设备身份、拓扑、真实 Heartbeat、状态、运动、故障与最终 Idle 必须由独立的实板记录证明。`Submitted`、单元测试、交叉编译、USB 枚举和刷写成功均不替代上述证据。
