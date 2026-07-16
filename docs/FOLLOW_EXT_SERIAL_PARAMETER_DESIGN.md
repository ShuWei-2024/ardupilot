# FOLLOW_EXT 串口参数设置方案

## 1. 目标

在保留现有伴随计算机串口帧格式的前提下，完善 `command content = 0x02` 的参数命令，使伴随计算机能够：

- 设置 `ModeFollowExt` 的自定义参数；
- 选择仅本次上电有效，或写入 ArduPilot 参数存储并在重启后继续生效；
- 读取参数当前值；
- 通过带序号的应答确认参数是否真正应用成功；
- 对未知参数、非法值、协议错误和队列繁忙给出明确错误码。

本文只设计 `FOLE_*` 白名单参数，不允许串口端借此修改任意 ArduPilot 参数。

当前实现采用 64 字节最大接收帧长度，并通过静态断言保证原有控制接收帧继续保持 36 字节。`CC_PORT` 按当前使用方式暂不修改。

## 2. 当前实现分析

### 2.1 数据链路

当前调用链如下：

```text
串口（SERIALx_PROTOCOL=50）
        │
        ▼
AP_CompanionComputer::update()                 50 Hz
        │
        ├── 0x01：解析 CompanionReceivePacket
        ├── 0x02：解析参数请求并放入固定队列
        └── 0x03：系统控制
        │
        ▼
Copter::receive_companion_computer()
        │
        ├── 消费参数命令并发送真实处理结果
        └── 处理 new_mode_flag
        │
        ▼
ModeFollowExt::run()
        └── 从 AP_CompanionComputer 读取最新控制包
```

`AP_CompanionComputer` 在 `Copter` 中以 `CC_` 参数组注册，已有：

- `CC_ENABLE`：启用通信；
- `CC_PORT`：设计上表示协议 50 串口实例，但当前 `init()` 固定查找实例 0，没有使用该值。

`ModeFollowExt` 在 `ParametersG2` 中以 `FOLE_` 参数组注册。因此 `mode_follow_ext.cpp` 中定义的参数最终名称为：

| 参数 ID | ArduPilot 参数名 | 类型 | 默认值 | 当前用途 |
|---:|---|---|---:|---|
| `0x01` | `FOLE_AUTO_ENABLE` | `AP_Int8` | 1 | 控制是否允许进入 `FOLLOW_EXT` |
| `0x02` | `FOLE_KP_YAW` | `AP_Float` | 0.05 | 计算偏航角速度 |
| `0x03` | `FOLE_KP_THR` | `AP_Float` | 0.1 | 计算爬升速度 |
| `0x04` | `FOLE_KD_YAW` | `AP_Float` | 0.0 | 已定义，但当前控制律没有使用 |
| `0x05` | `FOLE_KD_THR` | `AP_Float` | 0.0 | 已定义，但当前控制律没有使用 |
| `0x06` | `FOLE_SPEED` | `AP_Float` | 1000 | 前向速度，单位 cm/s |
| `0x07` | `FOLE_ALPHA` | `AP_Float` | 1.0 | 误差低通滤波系数 |

### 2.2 当前参数预留代码

`mode_follow_ext.cpp` 已经有一段被注释的预留逻辑：检测 `is_new_param()`，读取 `Mode1Param`，然后调用各参数的 `set_and_save()`。这说明原始设计方向是：通信库保存待处理参数，飞行模式负责应用参数。

不建议直接恢复这段代码，原因是：

1. 参数只有在 `FOLLOW_EXT` 正在运行且 `ctrl_mode == 1` 时才会应用；停在其他模式时无法设置。
2. 通信层在参数真正校验和应用前就回复成功。
3. `Mode1Param` 是一次覆盖全部参数，无法单独设置一个参数，也无法区分“未提供”和“设置为 0”。
4. 每次都调用 `set_and_save()` 会产生不必要的参数存储写入。
5. 单个 `_new_param_flag` 无法可靠承载连续到达的多条命令。

## 3. 推荐架构

采用“通信层负责收发和排队，模式对象负责参数语义”的分层方案：

```text
参数请求帧
   │
   ▼
AP_CompanionComputer
   ├── 校验帧格式、版本、操作码
   └── ParamCommand 固定长度队列（建议 4 项）
                         │
                         ▼
Copter::receive_companion_computer()（50 Hz）
   ├── 取出命令
   ├── ModeFollowExt::handle_external_param()
   │      ├── ID 白名单
   │      ├── 数值范围/有限值校验
   │      ├── set() 或 set_and_save()
   │      └── 返回实际值和状态
   └── AP_CompanionComputer::send_parameter_response()
```

这样即使飞行器当前不在 `FOLLOW_EXT` 模式，参数也可以设置；应答代表参数已经实际处理，而不是仅代表收到了数据包。

不建议在 `AP_CompanionComputer` 中通过任意字符串调用 `AP_Param::find()`。使用固定参数 ID 和 `ModeFollowExt` 的显式接口更安全，也不会把整个飞控参数表暴露给自定义串口。

## 4. 串口参数协议

### 4.1 通用帧

继续沿用现有格式：

| 字节 | 字段 | 参数请求值 |
|---:|---|---|
| 0 | 帧头 1 | `0xA5` |
| 1 | 帧头 2 | `0x5A` |
| 2 | 来源 | 请求为 `0xAA`，应答为 `0xBB` |
| 3 | 命令类型 | 参数命令固定为 `0x02` |
| 4 | `DATA_LENGTH` | 仅表示 payload 长度 |
| 5... | payload | 见下文 |
| 倒数第 2 字节 | checksum | 字节 2 到 payload 最后一个字节之和的低 8 位 |
| 倒数第 1 字节 | 结束符 | `0xFF` |

所有多字节整数和 `float32` 明确使用小端序。解析时应逐字节解码或 `memcpy` 到对齐安全的局部变量，不能把接收缓冲区直接 `reinterpret_cast<float *>`。

### 4.2 参数请求 payload

请求 payload 固定为 8 字节，`DATA_LENGTH = 0x08`：

| payload 偏移 | 帧字节 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 5 | `version` | 协议版本，首版为 `0x01` |
| 1 | 6 | `operation` | 见操作码表 |
| 2 | 7 | `sequence` | 请求序号，原样带回应答 |
| 3 | 8 | `param_id` | 参数 ID，见第 2.1 节 |
| 4...7 | 9...12 | `value` | IEEE-754 `float32`，小端序；GET 时填 0 |

操作码：

| 值 | 名称 | 行为 |
|---:|---|---|
| `0x01` | `SET_VOLATILE` | 修改 RAM 中的参数值，重启后恢复已保存值 |
| `0x02` | `SET_PERSISTENT` | 修改参数并使用 `set_and_save()` 持久化 |
| `0x03` | `GET` | 读取参数当前实际值 |

所有参数在协议中统一用 `float32` 传输。`FOLE_AUTO_ENABLE` 只接受精确的 `0.0` 或 `1.0`，应用时转换成 `int8_t`。

### 4.3 参数应答 payload

应答 payload 固定为 9 字节，`DATA_LENGTH = 0x09`：

| payload 偏移 | 字段 | 说明 |
|---:|---|---|
| 0 | `version` | `0x01` |
| 1 | `operation_ack` | 请求操作码按位或 `0x80` |
| 2 | `sequence` | 对应请求序号 |
| 3 | `param_id` | 对应参数 ID |
| 4 | `status` | 执行结果 |
| 5...8 | `actual_value` | 处理后的当前实际值，`float32` 小端序 |

状态码：

| 值 | 名称 | 含义 |
|---:|---|---|
| `0x00` | `OK` | 成功 |
| `0x01` | `BAD_VERSION` | 不支持的协议版本 |
| `0x02` | `BAD_OPERATION` | 不支持的操作码 |
| `0x03` | `UNKNOWN_PARAM` | 参数 ID 不在白名单 |
| `0x04` | `INVALID_VALUE` | NaN、Inf、非整数布尔值或超出范围 |
| `0x05` | `NOT_SUPPORTED` | 参数存在，但当前控制实现尚不支持其功能 |
| `0x06` | `SAVE_FAILED` | 参数未能持久化 |
| `0x07` | `BUSY` | 参数命令队列已满 |
| `0x08` | `MALFORMED` | payload 长度等格式错误 |

建议成功 SET 后返回再次读取到的实际值。这样发送端可以发现类型转换、限幅或写入失败；本方案建议非法值直接拒绝，不静默限幅。

### 4.4 报文示例

将 `FOLE_ALPHA` 持久化设置为 `0.5`，请求序号 `0x2A`：

```text
A5 5A AA 02 08  01 02 2A 07 00 00 00 3F  27 FF
```

校验和计算范围为：

```text
AA + 02 + 08 + 01 + 02 + 2A + 07 + 00 + 00 + 00 + 3F = 0x127
低 8 位 = 0x27
```

成功应答：

```text
A5 5A BB 02 09  01 82 2A 07 00 00 00 00 3F  B9 FF
```

读取 `FOLE_SPEED`，请求序号 `0x2B`：

```text
A5 5A AA 02 08  01 03 2B 06 00 00 00 00  E9 FF
```

若当前值为 `1000.0f`（小端字节 `00 00 7A 44`），成功应答为：

```text
A5 5A BB 02 09  01 83 2B 06 00 00 00 7A 44  39 FF
```

## 5. 参数校验建议

以下范围作为第一版安全边界。实际投入飞行前，应根据视觉误差单位和实机调参结果进一步收紧：

| 参数 | 建议接受范围 | 处理建议 |
|---|---:|---|
| `FOLE_AUTO_ENABLE` | 0 或 1 | 增加实际启停语义后再开放远程设置 |
| `FOLE_KP_YAW` | -1000.0 ～ 1000.0 | 必须是有限值 |
| `FOLE_KP_THR` | -1000.0 ～ 1000.0 | 必须是有限值；负增益会反转高度误差控制方向 |
| `FOLE_KD_YAW` | 0.0 ～ 5.0 | 当前未使用；建议返回 `NOT_SUPPORTED` |
| `FOLE_KD_THR` | 0.0 ～ 5.0 | 当前未使用；建议返回 `NOT_SUPPORTED` |
| `FOLE_SPEED` | 0 ～ 10000 cm/s | 与当前速度异常检查上限保持一致 |
| `FOLE_ALPHA` | 0.0 ～ 1.0 | 当前 `run()` 虽有限幅，设置入口仍应拒绝越界值 |

当前 `_followext_enabled` 已在 `ModeFollowExt::init()` 开头检查；设置为 0 后将拒绝下一次进入 `FOLLOW_EXT`。它不会强制退出已经在运行的模式。

当前两个 KD 参数没有进入控制律。第一版协议可以保留 ID，但 SET 返回 `NOT_SUPPORTED`；等微分项真正实现后再改为可写。也可以允许保存它们，但文档和应答必须明确“保存成功不代表当前控制生效”，前一种做法更不容易误用。

## 6. 代码修改方案

### 6.1 `AP_CompanionComputer_config.h`

新增协议枚举和结构体，不再使用整组 `Mode1Param`：

```cpp
enum class CompanionParamOperation : uint8_t {
    SET_VOLATILE   = 0x01,
    SET_PERSISTENT = 0x02,
    GET            = 0x03,
};

enum class CompanionParamStatus : uint8_t {
    OK            = 0x00,
    BAD_VERSION   = 0x01,
    BAD_OPERATION = 0x02,
    UNKNOWN_PARAM = 0x03,
    INVALID_VALUE = 0x04,
    NOT_SUPPORTED = 0x05,
    SAVE_FAILED   = 0x06,
    BUSY          = 0x07,
    MALFORMED     = 0x08,
};

struct CompanionParamCommand {
    uint8_t version;
    CompanionParamOperation operation;
    uint8_t sequence;
    uint8_t param_id;
    float value;
};
```

该结构体用于程序内部即可；线上报文仍建议按偏移显式编解码，避免依赖编译器布局。

### 6.2 `AP_CompanionComputer.h/.cpp`

增加：

- 深度为 4 的固定命令队列；
- `bool pop_parameter_command(CompanionParamCommand &cmd)`；
- `send_parameter_response(...)`；
- 小端 `float32` 编解码辅助函数；
- 接收统计计数器（可选）：格式错、校验错、队列满。

`parse_parameter_data()` 只做协议级校验和入队，不应提前发送成功 ACK。队列满时可立即回复 `BUSY`。

### 6.3 `ModeFollowExt` 接口

在 `mode.h` 中为 `ModeFollowExt` 增加公开接口：

```cpp
CompanionParamStatus handle_external_param(
    CompanionParamOperation operation,
    uint8_t param_id,
    float request_value,
    float &actual_value);
```

实现中使用 `switch (param_id)` 显式访问 `_kp_yaw` 等成员。SET 的通用步骤为：

1. 检查 `isfinite(request_value)`；
2. 检查该参数的允许范围；
3. `SET_VOLATILE` 调用参数对象的 `set()`；
4. `SET_PERSISTENT` 调用 `set_and_save()`，确保先临时设置、再持久化同一个值时也会真正写入存储；
5. 重新通过 `.get()` 读取并返回 `actual_value`。

GET 只返回 `.get()`，不修改参数。

`AP_Param` 的保存接口在这里不能向调用方提供完整的异步存储完成结果，因此 `SAVE_FAILED` 留作未来扩展。当前成功应答表示参数值已接受且保存已提交；不能把它解释为底层存储已经完成落盘。

### 6.4 `Copter.cpp`

在 `receive_companion_computer()` 的 `companion_computer.update()` 之后消费参数队列：

```cpp
CompanionParamCommand cmd;
while (companion_computer.pop_parameter_command(cmd)) {
    float actual_value = NAN;
    const auto status = mode_follow_ext.handle_external_param(
        cmd.operation, cmd.param_id, cmd.value, actual_value);
    companion_computer.send_parameter_response(cmd, status, actual_value);
}
```

建议限制单次调度最多处理 1～2 条参数命令，避免大量持久化请求占用 50 Hz 调度任务。发送端必须等待相同 `sequence` 的应答后再发下一条持久化命令。

### 6.5 删除旧的参数交接逻辑

完成新接口后删除或废弃：

- `Mode1Param`；
- `_param`；
- `_new_param_flag`；
- `is_new_param()`、`clear_new_param_flag()`、`get_mode1_param()`；
- `mode_follow_ext.cpp` 中被注释的整组参数保存代码。

## 7. 必须同时修正的接收器问题

参数帧比当前 36 字节控制帧短。如果不先修复以下问题，合法参数帧也可能校验失败或造成越界。

### 7.1 校验和必须使用实际帧长

当前代码使用固定缓冲区大小：

```cpp
calculate_checksum(_rx_buffer.data(), _rx_buffer.size() - 2);
```

应改为按本次实际收到的字节数计算：

```cpp
calculate_checksum(_rx_buffer.data(), _rx_count - 2);
```

否则短帧的校验会把缓冲区中未使用或上一次报文残留的字节也加进去。

### 7.2 接收长度上限

在 `WAITING_LENGTH` 状态立即验证：

```text
data_length <= COMPANION_RECV_TOTAL_LENGTH - 7
```

并在每次写 `_rx_buffer[_rx_count++]` 前检查边界。超长帧应复位状态机并计数，不能继续写入。

更好的做法是把“最大接收帧长度”和“控制帧固定长度 36”拆成两个常量，因为新增的参数应答/请求长度与控制帧不同。（按最合适的来）

### 7.3 来源、命令和固定长度校验

- 请求来源必须为 `0xAA`；
- `0x01` 控制帧必须验证 payload 长度与 `CompanionReceivePacket` 一致；
- `0x02` 参数请求第一版必须为 8 字节；
- `0x03` 系统命令必须验证至少有 1 字节 payload；
- 超时、错误帧头、错误命令和错误长度都要清空本帧计数并回到等待帧头状态。

### 7.4 串口实例参数（这个先不做）

`CC_PORT` 当前没有实际使用，`init()` 固定调用：

```cpp
find_serial(AP_SerialManager::SerialProtocol_2CC, 0)
```

后续需要多路协议 50 串口时，再将第二个参数改为 `_port_index`。本次保持实例 0 不变，不影响当前单串口使用方式。

## 8. 与追踪模式本身相关的风险

这些问题不阻塞参数协议编码，但会影响实机行为，联调前必须确认：

1. `Copter::receive_companion_computer()` 中进入 `FOLLOW_EXT` 的 `set_mode(...)` 已被注释，因此串口 `ctrl_mode` 变化目前不会自动切换飞行模式，只会重置起飞状态。
2. `AP_CompanionComputer::parse_flight_control_data()` 把 `ctrl_mode = 0x05` 注释为“追踪模式”，但 `ModeFollowExt::run()` 的 case 5 实际会立即 disarm（上锁停桨）。两处定义冲突，必须统一；紧急停转不应与追踪模式共用编号。
3. 通信层注释中 `ctrl_mode` 的定义与 `ModeFollowExt::run()` 的实际 case 含义不完全一致，应建立唯一的 `enum class CompanionCtrlMode` 并在两处共同使用。
4. `ModeFollowExt::run()` 的 case 1 才是当前真正使用 `FOLE_KP_YAW`、`FOLE_KP_THR`、`FOLE_SPEED` 和 `FOLE_ALPHA` 的视觉定速追踪控制。
5. 参数写入不应依赖飞行模式是否正在运行；但飞行中修改增益或速度有突变风险。可在第一版限制“解锁后只允许易平滑参数的 volatile 设置”，或由伴随计算机在安全状态下调参。

## 9. 测试与验收

### 9.1 协议单元测试

至少覆盖：

- 正确的 SET_VOLATILE、SET_PERSISTENT、GET；
- 每个参数 ID 的边界值和越界值；
- NaN、正负 Inf；
- 未知版本、操作码和参数 ID；
- 错误 checksum、错误结束符、错误来源；
- payload 长度为 0、7、8、9 和超出缓冲区上限；
- 两帧连续粘包、半帧分多次到达、接收超时；
- 队列满时返回 BUSY，且不覆盖尚未处理的命令。

### 9.2 SITL/板端功能测试

1. 上电读取全部参数，确认与地面站显示一致。
2. 用 SET_VOLATILE 修改 `FOLE_ALPHA`，GET 应返回新值；重启后恢复旧的已保存值。
3. 用 SET_PERSISTENT 修改 `FOLE_ALPHA`，重启后 GET 仍返回新值。
4. 先进行 volatile 设置，再对相同值进行 persistent 设置，重启后确认该值已保存。
5. 不进入 `FOLLOW_EXT` 时设置参数，确认仍能成功。
6. 进入 `FOLLOW_EXT` case 1 后修改速度和增益，确认控制器读取的是参数对象新值。
7. 使用地面站读取 `FOLE_*`，确认串口设置与 MAVLink 参数系统看到的值一致。

### 9.3 实飞安全顺序

先在桨叶拆除状态验证串口和持久化，再在 SITL/HIL 验证模式逻辑，最后进行系留低速测试。首次实飞建议只开放 `FOLE_ALPHA` 和较小范围的 `FOLE_SPEED` volatile 修改；增益和持久化写入在降落、停桨后进行。

## 10. 推荐实施顺序

1. 修复可变帧长校验和接收边界，同时保持控制帧固定为 36 字节。
2. 统一 `ctrl_mode` 枚举，解决 `0x05` 语义冲突。
3. 添加参数协议结构、编解码、命令队列和应答。
4. 添加 `ModeFollowExt::handle_external_param()` 及参数范围校验。
5. 在 `Copter::receive_companion_computer()` 中完成命令消费和真实结果应答。
6. 删除旧的 `Mode1Param/new_param_flag` 方案。
7. 完成单元测试、SITL 测试和板端串口测试。

`CC_PORT` 的实际接线留到需要多路同协议串口时再处理。

按此方案实现后，`FOLE_*` 仍然是标准 ArduPilot 参数：地面站、MAVLink 和自定义串口看到的是同一份运行时值和持久化值，不需要在 `ModeFollowExt` 内维护第二套配置。
