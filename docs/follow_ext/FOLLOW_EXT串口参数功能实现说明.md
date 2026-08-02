# FOLLOW_EXT 串口参数功能实现说明

## 1. 文档目的

本文面向维护 ArduCopter 和导引通信代码的开发者，说明 `FOLLOW_EXT` 串口参数功能的代码结构、调用链、关键约束和扩展方法。

对应的线上协议见：

- `docs/飞控与导引协议v1.4-20251209.md`
- `docs/FOLLOW_EXT_SERIAL_PARAMETER_DESIGN.md`

## 2. 实现目标

本次代码实现允许导引计算机通过自定义 UART 协议读写 `ModeFollowExt` 的 `FOLE_*` 参数，同时满足：

- 原 36 字节飞行控制帧继续正常工作；
- 参数可以在未进入 `FOLLOW_EXT` 模式时配置；
- 区分临时修改和持久化修改；
- 参数经过模式对象校验后才返回成功；
- 通信库不暴露任意 ArduPilot 参数名；
- 连续到达的参数帧不会简单覆盖上一条命令。

## 3. 涉及文件

| 文件 | 职责 |
|---|---|
| `libraries/AP_CompanionComputer/AP_CompanionComputer_config.h` | 协议长度、版本、操作码、状态码和命令结构定义 |
| `libraries/AP_CompanionComputer/AP_CompanionComputer.h` | 参数队列和参数应答接口声明 |
| `libraries/AP_CompanionComputer/AP_CompanionComputer.cpp` | 串口状态机、参数帧解析、队列管理和应答编码 |
| `ArduCopter/mode.h` | 声明 `ModeFollowExt::handle_external_param()` |
| `ArduCopter/mode_follow_ext.cpp` | 参数白名单、范围校验、读取和写入 |
| `ArduCopter/Copter.cpp` | 在 50 Hz 通信任务中消费参数命令并发送处理结果 |

## 4. 完整调用链

```text
NCU 参数请求帧
    │
    ▼
AP_CompanionComputer::update()
    │ 逐字节读取 UART
    ▼
AP_CompanionComputer::process_received_data()
    │ 帧头、来源、类型、长度、超时、边界、checksum、结束位校验
    ▼
AP_CompanionComputer::parse_parameter_data()
    │ 解析 version / operation / sequence / param_id / float32
    │ 协议错误直接回复错误状态
    ▼
固定深度参数命令队列
    │
    ▼
Copter::receive_companion_computer()            50 Hz
    │ 每周期最多消费 2 条
    ▼
ModeFollowExt::handle_external_param()
    │ 参数 ID 白名单、范围和功能状态校验
    │ AP_Param::set() / set_and_save()
    ▼
AP_CompanionComputer::send_parameter_response()
    │ 返回 sequence / status / actual_value
    ▼
NCU
```

## 5. 帧长度与原控制协议兼容

### 5.1 两类长度常量

代码将原来含义混合的接收长度拆成：

```cpp
constexpr uint8_t COMPANION_CONTROL_RECV_TOTAL_LENGTH = 36;
constexpr uint8_t COMPANION_MAX_RECV_TOTAL_LENGTH = 64;
constexpr uint8_t COMPANION_MAX_RECV_DATA_LENGTH =
    COMPANION_MAX_RECV_TOTAL_LENGTH - 7;
```

- `COMPANION_CONTROL_RECV_TOTAL_LENGTH` 只描述原飞行控制帧。
- `COMPANION_MAX_RECV_TOTAL_LENGTH` 描述状态机缓冲区能够接收的最大变长帧。
- 最大数据体长度等于最大总长度减去 7 字节通用帧开销。

### 5.2 编译期兼容检查

```cpp
static_assert(sizeof(CompanionReceivePacket) == 36, ...);
static_assert(sizeof(CompanionSendPacket) == 29, ...);
static_assert(sizeof(float) == 4, ...);
```

这些检查用于防止结构体字段、对齐方式或目标平台浮点宽度变化后静默破坏线上协议。

控制帧解析还会同时验证：

```cpp
_data_len == COMPANION_RECV_DATA_LENGTH
_rx_count == COMPANION_CONTROL_RECV_TOTAL_LENGTH
```

因此新增参数帧不会被误当成 `CompanionReceivePacket` 反序列化。

## 6. 接收状态机修改

`process_received_data()` 仍沿用原来的逐字节状态机，但增加了以下保护。

### 6.1 来源和命令类型

- 接收来源必须为 `COMPANION_CMD_SOURCE_RCVR`，即 `0xAA`。
- 当前只接受指令类型 `0x01`、`0x02`、`0x03`。

### 6.2 长度保护

在进入数据接收状态前检查：

```cpp
data_length <= COMPANION_MAX_RECV_DATA_LENGTH
```

每次写入 `_rx_buffer` 前也会检查 `_rx_count` 是否超出数组边界。非法帧通过 `reset_rx_state()` 统一清理状态。

### 6.3 按实际帧长计算校验和

旧代码使用整个固定数组的长度计算 checksum。参数帧短于控制帧时，这会把未使用或上一帧残留的字节加入计算。

当前实现使用：

```cpp
calculate_checksum(_rx_buffer.data(), _rx_count - 2);
```

其中 `_rx_count - 2` 排除了 checksum 和结束位，并使求和函数只处理本帧真实数据。

### 6.4 控制包接收时间

每次成功解析长度正确、checksum 正确的 `0x01` 飞行控制帧后，通信库记录：

```cpp
_last_control_packet_ms = AP_HAL::millis();
```

`ModeFollowExt` 通过 `get_last_control_packet_ms()` 读取该时间。参数帧和系统控制帧不会刷新控制包时间，因此它们不能掩盖飞行控制链路中断。

## 7. 参数请求解析

`parse_parameter_data()` 负责协议层工作，不直接访问 `ModeFollowExt` 参数。

处理顺序：

1. 尽可能读取版本、操作码、序号和参数 ID，便于错误应答。
2. 检查数据体长度是否为 8，否则返回 `MALFORMED`。
3. 使用 `decode_float_le()` 显式解码小端 `float32`。
4. 检查协议版本，否则返回 `BAD_VERSION`。
5. 检查操作码，否则返回 `BAD_OPERATION`。
6. 尝试将命令加入队列；队列满时返回 `BUSY`。

这里不检查具体参数 ID 和数值范围，因为参数语义由 `ModeFollowExt` 所有，不应散落在通信库中。

## 8. 参数命令队列

`AP_CompanionComputer` 内部使用深度为 4 的固定环形队列：

```cpp
std::array<CompanionParamCommand, COMPANION_PARAM_QUEUE_SIZE> _param_queue;
uint8_t _param_queue_head;
uint8_t _param_queue_tail;
uint8_t _param_queue_count;
```

采用固定数组的原因：

- 不需要动态内存；
- 内存占用确定；
- 多个参数帧在一次 UART 更新中到达时不会互相覆盖；
- 队列满时可以明确返回 `BUSY`。

`pop_parameter_command()` 只负责先进先出地取出命令。当前 UART 更新和 Copter 命令消费运行在同一调度上下文，不需要额外锁。

## 9. Copter 层接线

`Copter::receive_companion_computer()` 由调度器以 50 Hz 调用。

参数处理位于 `companion_computer.update()` 之后，因此一次任务执行可以完成：收帧、入队、参数应用和应答。

每个周期最多处理 2 条命令：

```cpp
while (processed_commands < 2 &&
       companion_computer.pop_parameter_command(command)) {
    ...
}
```

该限制避免大量参数请求，尤其是持久化请求，长期占用飞控调度时间。剩余命令保留到下一次 50 Hz 周期处理。

## 10. ModeFollowExt 参数处理

### 10.1 为什么由模式对象处理

`ModeFollowExt` 直接拥有 `_kp_yaw`、`_speed` 等 `AP_Param` 成员，因此它最适合负责：

- 参数 ID 白名单；
- 参数范围；
- 参数是否已被当前控制律支持；
- 内部参数类型转换；
- 返回应用后的实际值。

通信库不通过字符串调用 `AP_Param::find()`，从而避免导引串口修改任意飞控参数。

### 10.2 公共接口

```cpp
CompanionParamStatus handle_external_param(
    CompanionParamOperation operation,
    uint8_t param_id,
    float request_value,
    float &actual_value);
```

无论成功或部分参数错误，接口都会尽可能通过 `actual_value` 返回当前值。

### 10.3 写入方式

```cpp
SET_VOLATILE   -> AP_Param::set()
SET_PERSISTENT -> AP_Param::set_and_save()
GET            -> AP_Param::get()
```

持久化操作使用 `set_and_save()`，而不是简单地先 `set()` 再调用 `set_and_save_ifchanged()`。这样可以保证以下流程正确：

1. 先 volatile 设置为某个值；
2. 再 persistent 设置为同一个值；
3. 该值仍会真正提交到参数存储。

保存由 ArduPilot 参数系统异步处理，因此成功应答表示保存请求已提交，不代表物理存储写入已经同步完成。

### 10.4 当前参数行为

| ID | 参数 | SET 行为 |
|---:|---|---|
| `0x01` | `FOLE_AUTO_ENABLE` | 只接受 0/1；0 会阻止下一次进入模式 |
| `0x02` | `FOLE_KP_YAW` | 接受 -1000.0～1000.0 |
| `0x03` | `FOLE_KP_THR` | 接受 -1000.0～1000.0；负值会反转控制方向 |
| `0x04` | `FOLE_KD_YAW` | 接受 -1000.0～1000.0 |
| `0x05` | `FOLE_KD_THR` | 返回 `NOT_SUPPORTED`，GET 可用 |
| `0x06` | `FOLE_SPEED` | 接受 0～10000 cm/s |
| `0x07` | `FOLE_ALPHA` | 接受 0.0～1.0 |
| `0x08` | `FOLE_ERR_SLOW_EN` | 只接受 0 或 1 |
| `0x09` | `FOLE_TURN_LIM_EN` | 只接受 0 或 1 |
| `0x0A` | `FOLE_CLB_SPD_EN` | 只接受 0 或 1 |
| `0x0B` | `FOLE_TURN_FF_EN` | 只接受 0 或 1 |
| `0x0C` | `FOLE_YAW_D_EN` | 只接受 0 或 1 |
| `0x0D` | `FOLE_ERR_SLOW_SC` | 接受 0.001～100000.0 |
| `0x0E` | `FOLE_VERT_ERR_WT` | 接受 0.0～100.0 |
| `0x0F` | `FOLE_MIN_SPD_MUL` | 接受 0.0～1.0 |
| `0x10` | `FOLE_TURN_ACC_RT` | 接受 0.0～1.0 |
| `0x11` | `FOLE_MIN_YAW_RT` | 接受 0.0～10.0 rad/s |

所有 SET 都先检查 `isfinite()`。越界值直接拒绝，不进行静默限幅。

`FOLE_AUTO_ENABLE` 当前只在 `ModeFollowExt::init()` 入口检查。运行中将它设置为 0 不会强制退出当前模式。

### 10.5 控制包超时保护

`ModeFollowExt` 使用固定 500 ms 控制包超时：

```cpp
static constexpr uint32_t CONTROL_PACKET_TIMEOUT_MS = 500;
```

若从未收到控制包，或最后一个有效控制包已超过该时间，模式将：

1. 清零 `y_err` 和 `z_err`；
2. 向当前 `ModeFollowExt` 的 Guided 控制器提交零 NEU 速度；
3. 显式提交零偏航角速度；
4. 继续调用 `ModeGuided::run()`，使位置和姿态控制环保持运行；
5. 超时开始时发送一次 `FOLLOW_EXT: control timeout`；
6. 链路恢复时发送一次 `FOLLOW_EXT: control restored`。

这里不能只依赖 `GUID_TIMEOUT`。如果模式不断使用旧数据调用 `set_velocity()`，Guided 的内部更新时间会被持续刷新，原生超时永远不会发生。现在使用通信层独立记录的真实收包时间解决这一问题。

### 10.6 非法速度保护

速度命令提交前会检查每个分量：

```cpp
isfinite(velocity) && fabsf(velocity) <= 10000.0f
```

发现非法分量后，代码不再调用独立的 `copter.mode_guided.init()`，因为它不是当前运行的 `ModeFollowExt` 对象。当前实现会：

- 向本模式提交零速度和零偏航角速度；
- 跳过异常追踪命令；
- 继续运行当前 Guided 控制环；
- 每次故障持续期间只发送一次错误提示，避免 GCS 消息刷屏。

## 11. 参数应答

`send_parameter_response()` 构造固定 16 字节应答：

```text
A5 5A BB 02 09
version operation_ack sequence param_id status actual_value[4]
checksum FF
```

应答操作码为请求操作码按位或 `0x80`：

| 请求 | 应答 |
|---:|---:|
| `0x01` | `0x81` |
| `0x02` | `0x82` |
| `0x03` | `0x83` |

`sequence` 用于发送端匹配请求和应答。参数错误应答通常返回当前参数值；无法确定参数时返回 NaN。

参数应答与旧系统反馈都使用指令类型 `0x02`，但数据长度不同：

- 参数应答：`DATA_LENGTH = 9`；
- 系统反馈：`DATA_LENGTH = 2`。

## 12. 如何新增一个串口参数

假设需要新增 `FOLE_MAX_CLIMB`：

1. 在 `ModeFollowExt::var_info[]` 中添加新的 `AP_GROUPINFO`，使用未占用且永久稳定的参数索引。
2. 在 `ModeFollowExt` 类中添加对应的 `AP_Float` 或整数参数成员。
3. 在协议文档中分配新的 `param_id`。已经发布的 ID 不得改变含义。
4. 在 `handle_external_param()` 的 `switch` 中添加该 ID，并定义：
   - GET 行为；
   - SET 范围；
   - volatile/persistent 写入；
   - 返回的实际值。
5. 更新导引端参数 ID 表。
6. 添加边界值、越界值、GET、volatile、persistent 和重启保持测试。

不要直接把串口参数 ID 当作 `AP_Param` 内部索引。两者应保持独立，以免 ArduPilot 参数表调整破坏线上协议。

## 13. 开发和联调注意事项

### 13.1 发送端节流

- 普通参数请求应等待对应序号的应答。
- 收到 `BUSY` 后延时重试。
- persistent 设置只能低频使用，不能作为实时控制数据。

### 13.2 当前保留事项

- `CC_PORT` 仍未接入 `_port_index`，当前固定使用协议 50 的实例 0。
- `FOLE_KD_YAW` 和 `FOLE_KD_THR` 尚未进入控制律。
- 系统重启和关机命令的实际执行仍为预留代码。
- `ctrl_mode = 0x05` 在模式实现中执行急停/disarm；通信层相关注释应继续保持与该行为一致。
- 串口控制模式变化不会自动切换到 `FOLLOW_EXT`，因为 `set_mode()` 仍被注释。

### 13.3 验证命令

SITL 构建：

```bash
CCACHE_DIR=/tmp/ardupilot-ccache ./waf configure --board sitl
CCACHE_DIR=/tmp/ardupilot-ccache ./waf copter
```

本功能实现已通过 SITL ArduCopter 完整编译和链接。实机上线前仍应执行 UART 回环、粘包/拆包、错误 checksum、越界参数、重启保持和飞行安全测试。
