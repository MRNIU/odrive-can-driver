<!-- Copyright The odrive-can-driver Contributors -->
<!-- 本文件说明贡献者需要遵守的公共接口和兼容性边界。 -->

# 贡献指南

欢迎提交 [Pull Request](https://github.com/MRNIU/odrive-can-driver/pulls)。问题应给出最小复现、库的版本或提交，以及 Rust 或 C 编译器版本。CAN 问题还应给出固件版本、节点 ID、帧类型、CAN ID、DLC、有效载荷和观察时间。

请遵守 [README](README.md) 的支持范围、后端要求和操作语义。`Driver` 只记录本地 I/O 进展与协议观察；共享总线调度、外设配置、机械限值、保护许可和设备恢复仍由应用负责。`protocol` 只负责按固件版本编解码与帧适配；新增后端不等同于新增固件支持。

新增固件版本须用固定 revision 的官方文档和实现核对命令号、方向、帧形态、有效长度、端序、字段宽度与单位，并更新 README 的支持矩阵及源码依据。未知状态和错误位是协议数据，不能丢弃或改为解码错误。

新增或改写的说明性注释使用中文。Rust 的 rustdoc 和 C 公开头文件的 Doxygen 注释应说明时间单位、watchdog 影响和结果语义。C 文件用 `@file` 说明职责，公共类型、枚举值和成员各自说明含义；函数用 `@param[in]`、`@param[out]` 或 `@param[in,out]` 标明方向，对有返回值的函数写明返回语义，可用 `@retval` 说明各状态。指针的可空性、借用期限、失败时输出和需要调用方提供的证据在相应接口处说明。保留 MIT 许可及适用的原版权。

时间统一使用微秒 `Instant` / `Duration`，区分事件发生时间和处理时间。后端每次真实发送轮询前必须检查凭证资格，等待期间不得独占 Driver。声明“取消且未提交”须给出固定后端实现的源码依据，并证明原发送 future 已结束、不会继续提交；普通取消和超时不是证据。接收必须保留原生帧、原始错误、完整时间戳和被置换的 TX 帧；查询接纳规则只能收紧节点、类型与时间匹配。

## 验证

PR 说明触发条件、变更后行为、兼容性影响和风险边界。操作语义变化须同步 README、两种语言的文档与行为测试。协议固定字节与操作轨迹的独立预期放在 [tests/fixtures](tests/fixtures/)；先核对固件依据或操作合同，再更新预期。C 可复制凭证的重放用例放在 C 测试中，Rust 的编译期所有权用例保留在 rustdoc。

在仓库根目录运行以下命令可执行宿主测试。CMake 作为子目录引入时，只向消费工程提供静态库。

```sh
cargo test --locked --all-targets
cargo test --locked --doc
cmake -S . -B /tmp/odrive-can-build
cmake --build /tmp/odrive-can-build
ctest --test-dir /tmp/odrive-can-build --output-on-failure
```

运行 `doxygen Doxyfile` 检查 C 公开 API 的注释并生成 `build/doxygen/index.html`。独立消费工程与 Cortex-M7 的交叉链接检查见 [CI](.github/workflows/ci.yml)。
