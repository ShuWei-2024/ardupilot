# FOLLOW_EXT 视觉误差与转向需求前速调度说明

## 1. 功能目的

视觉导引模式使用两个独立开关动态调整无人机的前向速度。开启后，前速不再始终等于 `FOLE_SPEED`，而是可以依次受到以下两项限制：

1. 目标偏离画面中心时，根据横向和垂向视觉误差降低前速。
2. 无人机转向时，根据偏航角速度和水平加速度能力进一步限制前速。

该功能的主要目标是避免高速状态下目标越偏越难追踪，以及避免前速和偏航角速度组合产生超出飞行器能力的向心加速度。

功能开关及调度常量定义在 `ArduCopter/mode_follow_ext.cpp` 顶部：

```cpp
static constexpr bool FOLLOW_EXT_ENABLE_ERROR_SLOWDOWN = true;
static constexpr bool FOLLOW_EXT_ENABLE_TURN_ACCEL_LIMIT = true;

static constexpr float FOLLOW_EXT_ERROR_SLOWDOWN_SCALE = 300.0f;
static constexpr float FOLLOW_EXT_VERTICAL_ERROR_WEIGHT = 0.5f;
static constexpr float FOLLOW_EXT_MIN_SPEED_MULTIPLIER = 0.0f;
static constexpr float FOLLOW_EXT_TURN_ACCEL_BUDGET_RATIO = 0.6f;
static constexpr float FOLLOW_EXT_MIN_YAW_RATE_RAD_S = radians(2.0f);
```

`FOLLOW_EXT_ENABLE_ERROR_SLOWDOWN` 单独控制视觉误差减速，`FOLLOW_EXT_ENABLE_TURN_ACCEL_LIMIT` 单独控制转向加速度限速。两个开关都设为 `false` 后，前速恢复为 `FOLE_SPEED`；横向加速度前馈仍由它自己的开关独立控制。

| 视觉误差减速 | 转向加速度限速 | 控制效果 |
|---|---|---|
| 开 | 开 | 正常推荐配置，同时考虑目标偏差和转弯能力 |
| 开 | 关 | 只根据目标偏差减速，不约束 `v*|ω|` |
| 关 | 开 | 不根据画面误差减速，但仍受转弯加速度能力约束 |
| 关 | 关 | 保持基础前速，不执行这两项调度 |

## 2. 总体计算流程

前速按照以下顺序计算：

```text
FOLE_SPEED
    │
    ▼
根据视觉综合误差计算速度倍率
    │
    ▼
得到视觉误差限制后的前速
    │
    ▼
根据 v × |偏航角速度| 检查转弯加速度
    │
    ▼
得到最终前向速度指令
```

最终关系可以表示为：

\[
v_{cmd} = \min\left(v_{base}K_{error},\frac{a_{budget}}{|\omega|}\right)
\]

当偏航角速度没有达到转向限速阈值时，只使用视觉误差减速：

\[
v_{cmd}=v_{base}K_{error}
\]

## 3. 视觉综合误差

代码首先将横向误差和垂向误差组合为一个综合误差：

```cpp
const float error_magnitude = safe_sqrt(
    sq(y_err) + FOLLOW_EXT_VERTICAL_ERROR_WEIGHT * sq(z_err));
```

对应公式：

\[
E=\sqrt{e_y^2+w_z e_z^2}
\]

其中：

- \(e_y\) 为 `y_axis_err` 经过低通滤波后的横向误差。
- \(e_z\) 为 `z_axis_err` 经过低通滤波后的垂向误差。
- \(w_z\) 为 `FOLLOW_EXT_VERTICAL_ERROR_WEIGHT`。
- \(E\) 为综合视觉误差。

误差经过平方后，正负方向不会影响减速程度。例如目标在画面左侧或右侧相同距离时，前速降低程度相同。

当前垂向权重为 `0.5`：

\[
E=\sqrt{e_y^2+0.5e_z^2}
\]

因此，在误差数值相同的情况下，垂向误差对前速的影响小于横向误差。

示例：

| `y_err` | `z_err` | 综合误差 |
|---:|---:|---:|
| 100 | 0 | 100.0 |
| 0 | 100 | 70.7 |
| 100 | 100 | 122.5 |

## 4. 视觉误差速度倍率

综合误差首先除以误差减速尺度：

```cpp
const float error_ratio =
    error_magnitude / FOLLOW_EXT_ERROR_SLOWDOWN_SCALE;
```

即：

\[
r=\frac{E}{S}
\]

其中 \(S\) 为 `FOLLOW_EXT_ERROR_SLOWDOWN_SCALE`。

随后计算速度倍率：

```cpp
const float error_speed_multiplier = constrain_float(
    1.0f / (1.0f + sq(error_ratio)),
    FOLLOW_EXT_MIN_SPEED_MULTIPLIER,
    1.0f);
```

对应公式：

\[
K_{error}=\operatorname{constrain}
\left(
\frac{1}{1+(E/S)^2},
K_{min},
1
\right)
\]

误差减速后的前速为：

\[
v_{error}=v_{base}K_{error}
\]

其中 \(v_{base}\) 当前为 `FOLE_SPEED`。

### 4.1 速度倍率特性

当 `FOLLOW_EXT_ERROR_SLOWDOWN_SCALE = 300` 时：

| 综合误差 | `E/S` | 速度倍率 | `FOLE_SPEED=1000cm/s` 时的前速 |
|---:|---:|---:|---:|
| 0 | 0.0 | 1.00 | 1000cm/s |
| 150 | 0.5 | 0.80 | 800cm/s |
| 300 | 1.0 | 0.50 | 500cm/s |
| 600 | 2.0 | 0.20 | 200cm/s |
| 900 | 3.0 | 0.10 | 100cm/s |

该函数在小误差区域变化比较平缓，可以减少视觉噪声引起的速度波动；当误差接近或超过减速尺度时，前速会明显降低。

### 4.2 最小速度倍率

`FOLLOW_EXT_MIN_SPEED_MULTIPLIER` 只限制视觉误差减速阶段的最低倍率。

例如：

```cpp
static constexpr float FOLLOW_EXT_MIN_SPEED_MULTIPLIER = 0.2f;
```

表示无论视觉误差多大，经过视觉误差调度后的前速都不会低于：

\[
0.2v_{base}
\]

需要注意，后续的转向加速度限速仍可将最终前速降低到该倍率以下。这是为了保证转弯所需加速度不超过设定预算。

## 5. 转向加速度限速

无人机以速度 \(v\) 和偏航角速度 \(\omega\) 转弯时，所需向心加速度近似为：

\[
a_{turn}=v|\omega|
\]

单位关系为：

```text
v：cm/s
ω：rad/s
a_turn：cm/s²
```

控制器没有将全部水平加速度能力都分配给转弯，而是保留一部分裕度用于速度误差修正、抗风和避障：

```cpp
const float turn_accel_budget_cmss =
    FOLLOW_EXT_TURN_ACCEL_BUDGET_RATIO *
    pos_control->get_max_accel_xy_cmss();
```

对应公式：

\[
a_{budget}=K_a a_{xy,max}
\]

其中：

- \(K_a\) 为 `FOLLOW_EXT_TURN_ACCEL_BUDGET_RATIO`。
- \(a_{xy,max}\) 为位置控制器允许的最大水平加速度。
- \(a_{budget}\) 为分配给转弯的水平加速度预算。

为了满足：

\[
v|\omega|\leq a_{budget}
\]

允许的最大转弯前速为：

\[
v_{turn,max}=\frac{a_{budget}}{|\omega|}
\]

代码使用视觉误差调度速度和转弯允许速度中的较小值：

```cpp
forward_speed_cms = MIN(
    forward_speed_cms,
    turn_accel_budget_cmss / fabsf(yaw_rate_rad_s));
```

即：

\[
v_{cmd}=\min(v_{error},v_{turn,max})
\]

### 5.1 转向限速示例

假设位置控制器最大水平加速度为 `250cm/s²`，转弯预算比例为 `0.6`：

\[
a_{budget}=250\times0.6=150cm/s^2
\]

不同偏航角速度下允许的最大转弯前速如下：

| 偏航角速度 | 偏航角速度 rad/s | 最大转弯前速 |
|---:|---:|---:|
| 5°/s | 0.087 | 1724cm/s |
| 10°/s | 0.175 | 859cm/s |
| 20°/s | 0.349 | 430cm/s |
| 25°/s | 0.436 | 344cm/s |

例如，`FOLE_SPEED` 为 `1000cm/s`、偏航角速度为 `25°/s` 时，如果仍保持全部前速，需要的向心加速度为：

\[
1000\times0.436=436cm/s^2
\]

该值明显超过示例中的 `150cm/s²` 转弯预算，因此控制器会将前速限制到约 `344cm/s`。

## 6. 最小偏航角速度阈值

当 `FOLLOW_EXT_ENABLE_TURN_ACCEL_LIMIT` 开启时，转向限速只在偏航角速度超过以下阈值后生效：

```cpp
static constexpr float FOLLOW_EXT_MIN_YAW_RATE_RAD_S = radians(2.0f);
```

原因是转向速度上限包含除法：

\[
v_{turn,max}=\frac{a_{budget}}{|\omega|}
\]

当 \(\omega\) 接近零时，计算结果非常大，而且偏航角速度的微小噪声可能造成没有意义的速度变化。

当前逻辑为：

```text
|ω| ≤ 2.0°/s：不执行转向加速度限速
|ω| > 2.0°/s：执行转向加速度限速
```

## 7. 参数调节方法

### 7.1 调参前准备

当前通信协议没有定义 `y_axis_err` 和 `z_axis_err` 的单位。开始调参前，应通过日志确认以下场景中的实际误差范围：

- 目标位于画面中心。
- 正常跟踪时的典型误差。
- 目标位于画面一半位置附近。
- 目标接近画面边缘。
- 视觉检测短时抖动时的误差峰值。

如果伴随计算机改变图像分辨率、视场角或误差定义，这些参数通常需要重新调整。更理想的做法是让伴随计算机发送归一化误差。

### 7.2 推荐调节顺序

1. 暂时保持 `FOLLOW_EXT_TURN_ACCEL_BUDGET_RATIO = 0.6`。
2. 使用较低的 `FOLE_SPEED` 测试视觉误差减速。
3. 调整 `FOLLOW_EXT_ERROR_SLOWDOWN_SCALE`，使目标偏到画面约一半位置时前速降低到约一半。
4. 调整 `FOLLOW_EXT_VERTICAL_ERROR_WEIGHT`，确定高度误差是否需要明显影响前速。
5. 确定目标严重偏离时是否允许停止，并调整 `FOLLOW_EXT_MIN_SPEED_MULTIPLIER`。
6. 逐渐提高 `FOLE_SPEED`，观察转弯外扩、掉高和目标重新居中时间。
7. 最后调整 `FOLLOW_EXT_TURN_ACCEL_BUDGET_RATIO`。

### 7.3 各参数调节方向

#### `FOLLOW_EXT_ERROR_SLOWDOWN_SCALE`

- 增大：相同误差下速度更高，跟随更积极，但更容易冲过目标。
- 减小：更早减速，更容易重新对准目标，但平均速度降低。
- 建议每次按照当前值的 10%～20% 调整。

#### `FOLLOW_EXT_VERTICAL_ERROR_WEIGHT`

- `0`：垂向误差不影响前速。
- `1`：横向和垂向误差数值相同时，对前速影响相同。
- 升降过程中前速下降过多时应减小。
- 高度偏差较大但无人机仍持续高速前进时可适当增大。

#### `FOLLOW_EXT_MIN_SPEED_MULTIPLIER`

- `0`：视觉误差很大时允许接近停止并优先对准目标。
- `0.1`：至少保留基础前速的 10%。
- `0.2`：至少保留基础前速的 20%。
- 首次测试建议使用 `0～0.1`。

#### `FOLLOW_EXT_TURN_ACCEL_BUDGET_RATIO`

- 增大：允许更高速度转弯，但倾角、推力消耗和转弯外扩风险增加。
- 减小：转弯时减速更明显，轨迹更加保守。
- 强风、重载、动力不足或转弯掉高时，建议降低到 `0.4～0.5`。
- 动力充足并且日志显示仍有明显加速度余量时，可逐步提高到 `0.7`。
- 不建议设置到接近 `1.0`，因为这样几乎不再为抗风和速度修正保留余量。

#### `FOLLOW_EXT_MIN_YAW_RATE_RAD_S`

- 增大：只有明显转向时才触发转向限速，可减少直线飞行时的前速波动。
- 减小：轻微转向也会参与限速，控制更保守。
- 当前使用 `2.0°/s`，可过滤直线飞行时的小偏航角速度波动。
- 如果希望轻微转向也开始限速，可逐步降低到 `0.5～1.0°/s`。

## 8. 与横向加速度前馈的关系

当 `FOLLOW_EXT_ENABLE_TURN_ACCEL_FEEDFORWARD` 开启时，控制器使用相同的运动学关系生成横向加速度前馈：

\[
a_{ff}=v_{cmd}\omega
\]

误差与转向减速先确保 `v_cmd` 不会要求超过预算的向心加速度，随后横向加速度前馈将该转弯需求直接传递给 Guided 位置控制器。

因此两个功能配合后的控制关系为：

```text
视觉误差和偏航角速度决定安全前速
              ↓
使用最终前速计算 v × ω 横向加速度前馈
              ↓
Guided 同时接收速度、加速度和偏航角速度指令
```

即使关闭横向加速度前馈，视觉误差与转向减速仍可独立工作；反之亦然。

## 9. 注意事项

- `FOLLOW_EXT_MIN_SPEED_MULTIPLIER` 不是最终速度的绝对下限，转向加速度限制可以继续降低前速。
- 如果视觉误差单位是像素，改变图像分辨率后应重新调整误差减速尺度。
- 如果视觉误差存在明显跳变，应先调整低通滤波和检测稳定性，再提高减速灵敏度。
- 水平加速度预算来自位置控制器配置，因此修改位置控制器最大加速度也会改变转向限速结果。
- 首次实飞应使用较低 `FOLE_SPEED`，并检查急转弯时的倾角、垂向速度、推力余量和目标丢失情况。
- 调参时应一次只修改一个常量，并保存对应日志用于比较。

## 10. 1080×720 像素误差配置示例

本节给出一个具体的相机和视觉误差配置示例。假设：

- 图像宽度为 `1080px`，高度为 `720px`。
- 视觉系统已经将图像中心转换为坐标原点 `(0,0)`。
- `y_axis_err` 表示水平方向像素误差，目标在右侧时为正。
- `z_axis_err` 表示垂直方向像素误差，目标在下方时为正。

此时理论误差范围约为：

```text
y_axis_err：-540 ～ +540px
z_axis_err：-360 ～ +360px
```

如果视觉系统发送的是目标在原始图像中的像素坐标，应先在伴随计算机端转换：

```text
y_axis_err = target_pixel_x - 540
z_axis_err = target_pixel_y - 360
```

在开始飞行前，必须低速确认两个误差的正负方向与飞控控制方向一致。

### 10.1 推荐初始常量

对于上述像素误差定义，可以先采用：

```cpp
static constexpr float FOLLOW_EXT_ERROR_SLOWDOWN_SCALE = 270.0f;
static constexpr float FOLLOW_EXT_VERTICAL_ERROR_WEIGHT = 2.25f;
static constexpr float FOLLOW_EXT_MIN_SPEED_MULTIPLIER = 0.05f;

static constexpr float FOLLOW_EXT_TURN_ACCEL_BUDGET_RATIO = 0.6f;
static constexpr float FOLLOW_EXT_MIN_YAW_RATE_RAD_S = radians(2.0f);
```

其中前三项与图像尺寸和像素误差定义有关，后两项主要与飞行器运动能力有关。

### 10.2 垂向误差权重的计算

从图像中心到水平边缘的距离为 `540px`，到垂直边缘的距离为 `360px`。如果希望目标到达水平边缘和垂直边缘时产生相同的综合误差，应设置：

\[
w_z=\left(\frac{540}{360}\right)^2=2.25
\]

此时：

\[
E=\sqrt{y_{err}^2+2.25z_{err}^2}
\]

水平边缘和垂直边缘对应的综合误差相同：

```text
目标位于水平边缘：(540, 0)   → E = 540
目标位于垂直边缘：(0, 360)   → E = 540
```

不同权重的含义如下：

| 垂向权重 | 控制效果 |
|---:|---|
| 0 | 垂向误差完全不影响前速 |
| 0.5 | 明显优先考虑横向误差，垂向偏差只引起较弱减速 |
| 1.0 | 按原始像素数计算，但没有补偿图像宽高差异 |
| 2.25 | 按中心到边缘的比例归一化，水平和垂直边缘影响相同 |

如果升降过程中前速下降过多，可以将 `2.25` 逐步减小到 `1.0` 或 `0.5`。如果希望目标高度没有对准前不要高速前进，则应保持较大的垂向权重。

### 10.3 误差减速尺度的选择

从图像中心到水平边缘的距离为 `540px`，中心与边缘之间的一半位置为：

\[
540/2=270px
\]

设置：

```cpp
static constexpr float FOLLOW_EXT_ERROR_SLOWDOWN_SCALE = 270.0f;
```

表示综合误差达到 `270` 时，误差速度倍率为 `0.5`：

\[
K_{error}=\frac{1}{1+(270/270)^2}=0.5
\]

使用 `FOLLOW_EXT_VERTICAL_ERROR_WEIGHT = 2.25` 时，典型位置与速度倍率如下：

| 目标位置 | 综合误差 | 速度倍率 |
|---|---:|---:|
| 图像中心 | 0 | 1.00 |
| 水平偏移135px | 135 | 0.80 |
| 水平偏移270px | 270 | 0.50 |
| 水平偏移540px | 540 | 0.20 |
| 垂直偏移180px | 270 | 0.50 |
| 垂直偏移360px | 540 | 0.20 |
| 图像角落 `(540,360)` | 约764 | 约0.11 |

如果 `FOLE_SPEED = 1000cm/s`，对应的前速约为：

| 目标位置 | 前向速度 |
|---|---:|
| 图像中心 | 1000cm/s，即10m/s |
| 水平或垂直半幅偏差 | 500cm/s，即5m/s |
| 水平或垂直边缘 | 200cm/s，即2m/s |
| 图像角落 | 111cm/s，即1.11m/s |

调节方向：

- 目标明显偏离后无人机仍然冲得太快：减小到 `200～240`。
- 正常跟随时前速下降过于频繁：增大到 `320～350`。
- 建议每次按照当前值的 10%～20% 调整。

### 10.4 最小前速倍率

首次测试建议：

```cpp
static constexpr float FOLLOW_EXT_MIN_SPEED_MULTIPLIER = 0.05f;
```

这表示视觉误差调度至少保留基础前速的 5%。例如：

```text
FOLE_SPEED = 1000cm/s
视觉误差调度最低速度 = 50cm/s
```

可根据任务需求选择：

| 最小倍率 | 控制效果 |
|---:|---|
| 0.00 | 目标严重偏离时允许接近停止，最保守 |
| 0.05 | 保留很小的前速，推荐初始值 |
| 0.10 | 至少保留基础前速的10% |
| 0.20 | 持续明显向前运动，目标偏离时风险较高 |

该倍率只限制视觉误差减速阶段。转向加速度限速仍然可以将最终前速降低到该倍率以下。

### 10.5 转向限速参数

以下参数一般不需要根据相机分辨率修改：

```cpp
static constexpr float FOLLOW_EXT_TURN_ACCEL_BUDGET_RATIO = 0.6f;
static constexpr float FOLLOW_EXT_MIN_YAW_RATE_RAD_S = radians(2.0f);
```

`FOLLOW_EXT_TURN_ACCEL_BUDGET_RATIO = 0.6` 表示最多使用位置控制器最大水平加速度的 60% 进行转弯，其余 40% 用于抗风、速度误差修正和避障。

- 转弯掉高、外扩、动力不足或重载时，可降低到 `0.4～0.5`。
- 转弯过于保守且动力和倾角余量充足时，可提高到 `0.65～0.7`。
- 一般不建议超过 `0.7`。

`FOLLOW_EXT_MIN_YAW_RATE_RAD_S = radians(2.0f)` 表示偏航角速度超过 `2°/s` 后才开始进行转向加速度限速，可以过滤直线飞行中的小角速度波动。

### 10.6 `FOLE_KP_YAW` 的计算

视觉模式的比例项关系为：

\[
\omega_{cmd}(cd/s)=KP_{YAW}\times y_{err}(px)
\]

如果希望目标位于水平边缘 `540px` 时达到指定偏航角速度，可使用：

\[
KP_{YAW}=\frac{\omega_{edge}(°/s)\times100}{540}
\]

典型结果：

| 水平边缘期望偏航角速度 | `FOLE_KP_YAW` |
|---:|---:|
| 5°/s | 0.93 |
| 10°/s | 1.85 |
| 15°/s | 2.78 |
| 20°/s | 3.70 |

原默认值 `FOLE_KP_YAW = 0.05` 在像素误差输入下通常偏小。目标位于水平边缘时，它只产生：

\[
0.05\times540=27cd/s=0.27°/s
\]

首次低速测试建议：

```text
FOLE_KP_YAW = 1.0～2.0
FOLE_KD_YAW = 0
```

确认偏航方向正确且没有明显振荡后，再逐渐提高 `FOLE_KP_YAW`。

### 10.7 `FOLE_KD_YAW` 的调节

偏航PD控制关系为：

\[
\omega_{cmd}=KP_{YAW}e_y+KD_{YAW}\frac{de_y}{dt}
\]

建议先使用 `FOLE_KD_YAW = 0` 完成比例项调节。如果目标在画面中心附近出现左右往复摆动，可从以下数值开始增加：

```text
FOLE_KD_YAW = 0.1
```

每次增加 `0.05～0.1`，观察误差收敛和偏航角速度变化。D项对像素跳变和检测噪声较敏感，出现偏航角速度尖峰时应减小D项、加强视觉滤波或减小 `FOLE_ALPHA`。

### 10.8 `FOLE_KP_THR` 的计算

固定垂向增益开启后，垂向速度关系为：

\[
v_z(cm/s)=KP_{THR}\times z_{err}(px)
\]

如果希望目标位于垂直边缘 `360px` 时达到垂向速度上限 `200cm/s`：

\[
KP_{THR}=\frac{200}{360}\approx0.56
\]

首次测试建议：

```text
FOLE_KP_THR = 0.3～0.5
```

例如使用 `FOLE_KP_THR = 0.4`：

| 垂向误差 | 垂向速度指令 |
|---:|---:|
| 100px | 40cm/s |
| 180px | 72cm/s |
| 360px | 144cm/s |

必须先在低速、低高度风险环境中确认 `z_axis_err` 符号。若目标在画面下方时无人机向错误方向升降，应先修正伴随计算机误差符号或飞控控制符号，不应依靠增益大小掩盖方向错误。

### 10.9 第一轮实飞建议配置

编译期常量：

```cpp
static constexpr float FOLLOW_EXT_ERROR_SLOWDOWN_SCALE = 270.0f;
static constexpr float FOLLOW_EXT_VERTICAL_ERROR_WEIGHT = 2.25f;
static constexpr float FOLLOW_EXT_MIN_SPEED_MULTIPLIER = 0.05f;
static constexpr float FOLLOW_EXT_TURN_ACCEL_BUDGET_RATIO = 0.6f;
static constexpr float FOLLOW_EXT_MIN_YAW_RATE_RAD_S = radians(2.0f);
```

飞控参数：

```text
FOLE_SPEED  = 300～500cm/s
FOLE_ALPHA  = 0.3～0.5
FOLE_KP_YAW = 1.0
FOLE_KD_YAW = 0
FOLE_KP_THR = 0.3
```

推荐测试顺序：

1. 固定无人机或保持低速悬停，确认 `y_axis_err` 和 `z_axis_err` 的正负方向。
2. 使用 `FOLE_KD_YAW = 0` 调整 `FOLE_KP_YAW`，先保证偏航能够稳定对准目标。
3. 调整 `FOLE_KP_THR`，保证垂向误差能够稳定收敛。
4. 检查目标处于中心、半幅、边缘时的前速是否符合预期。
5. 逐渐提高 `FOLE_SPEED`，观察转弯外扩、倾角、掉高和目标丢失情况。
6. 比例项基本稳定后，再根据振荡情况小幅加入 `FOLE_KD_YAW`。
7. 每次只修改一个参数，并保存对应日志用于对比。

## 11. 35m/s以上高速前飞配置说明

本节讨论目标基本居中时，以超过 `35m/s` 的地速前飞所需的参数和约束。需要注意：`FOLLOW_EXT` 当前控制的是NE坐标系地速，不是空速。顺风和逆风条件下，相同地速对应的真实空速和动力负载可能明显不同。

对于多旋翼，`35m/s` 已经属于非常高的速度。能否达到该速度不仅由参数决定，还取决于机体阻力、动力系统、桨叶效率、电池能力、倾角限制、定位质量、飞行空间和制动距离。

### 11.1 推荐起始参数

如果希望目标基本居中时以约 `40～45m/s` 飞行，并在一定视觉误差范围内仍保持 `35m/s` 以上，可以先设置：

```text
FOLE_SPEED  = 4500cm/s
WPNAV_SPEED = 4500～5000cm/s
WPNAV_ACCEL = 400～500cm/s²
```

编译期调度常量可先保持：

```cpp
static constexpr float FOLLOW_EXT_ERROR_SLOWDOWN_SCALE = 270.0f;
static constexpr float FOLLOW_EXT_VERTICAL_ERROR_WEIGHT = 2.25f;
static constexpr float FOLLOW_EXT_MIN_SPEED_MULTIPLIER = 0.05f;
static constexpr float FOLLOW_EXT_TURN_ACCEL_BUDGET_RATIO = 0.6f;
static constexpr float FOLLOW_EXT_MIN_YAW_RATE_RAD_S = radians(2.0f);
```

不要一开始就在实飞中直接使用上述最大目标，应按照后文的分阶段方案逐步提高速度。

### 11.2 为什么基础前速应高于35m/s

视觉误差减速关系为：

\[
v_{cmd}=\frac{v_{base}}{1+(E/S)^2}
\]

如果仅设置：

```text
FOLE_SPEED = 3500cm/s
```

那么只有综合视觉误差严格等于零时，速度才等于 `35m/s`。任何非零误差都会使最终前速低于 `35m/s`。

使用：

```text
FOLE_SPEED = 4500cm/s
FOLLOW_EXT_ERROR_SLOWDOWN_SCALE = 270
```

时：

| 综合视觉误差 | 速度倍率 | 最终前速 |
|---:|---:|---:|
| 0 | 1.000 | 45.0m/s |
| 100 | 0.879 | 39.5m/s |
| 135 | 0.800 | 36.0m/s |
| 144 | 0.779 | 约35.0m/s |
| 270 | 0.500 | 22.5m/s |

因此，使用 `FOLE_SPEED = 4500cm/s` 时，综合视觉误差需要保持在约 `144` 以内，才能维持 `35m/s` 以上。

若需要根据目标基础速度和最低期望速度计算允许的最大综合误差，可使用：

\[
E_{max}=S\sqrt{\frac{v_{base}}{v_{min}}-1}
\]

该公式只考虑视觉误差减速，不包含后续的转向加速度限速。

### 11.3 `WPNAV_SPEED`配置

Guided速度控制初始化时，会使用 `WPNAV_SPEED` 设置位置控制器的水平速度范围。因此建议：

\[
WPNAV\_SPEED\geq FOLE\_SPEED
\]

例如：

```text
FOLE_SPEED  = 4500cm/s
WPNAV_SPEED = 4500～5000cm/s
```

`FOLE_SPEED` 当前代码允许的设置范围为 `0～10000cm/s`，`WPNAV_SPEED` 参数支持更高的数值，因此代码数值范围本身可以容纳 `35m/s` 以上的目标速度。但参数允许设置并不代表具体飞行器一定能够达到。

### 11.4 `WPNAV_ACCEL`与加速、制动距离

默认水平加速度通常为：

```text
WPNAV_ACCEL = 250cm/s² = 2.5m/s²
```

忽略空气阻力并按恒定加速度估算，从静止加速到 `35m/s` 需要：

\[
t=\frac{35}{2.5}=14s
\]

理论加速距离为：

\[
d=\frac{35^2}{2\times2.5}=245m
\]

如果设置：

```text
WPNAV_ACCEL = 500cm/s² = 5m/s²
```

理论结果为：

```text
加速时间约7s
加速距离约122.5m
```

相同加速度能力下，从 `35m/s` 制动到停止也需要相近的理论距离，实际飞行还应为控制延迟、风扰和动力变化保留额外裕度。

建议从 `WPNAV_ACCEL = 300cm/s²` 开始，根据姿态、动力和日志逐渐提高到 `400～500cm/s²`。不要仅为了缩短加速时间直接设置最大值。

### 11.5 35m/s下的转弯限制

转向加速度限速满足：

\[
v|\omega|\leq K_a a_{xy,max}
\]

保持指定前速时允许的最大偏航角速度为：

\[
|\omega|_{max}=\frac{K_a a_{xy,max}}{v}
\]

#### `WPNAV_ACCEL = 250cm/s²`

使用转弯预算比例 `0.6`：

\[
a_{budget}=250\times0.6=150cm/s^2
\]

保持 `35m/s` 时：

\[
|\omega|_{max}=\frac{150}{3500}=0.0429rad/s\approx2.46°/s
\]

偏航角速度超过约 `2.46°/s` 后，转向限速会将前速降低到 `35m/s` 以下。

#### `WPNAV_ACCEL = 500cm/s²`

转弯预算为：

\[
a_{budget}=500\times0.6=300cm/s^2
\]

保持 `35m/s` 时：

\[
|\omega|_{max}=\frac{300}{3500}=0.0857rad/s\approx4.91°/s
\]

因此，即使把 `WPNAV_ACCEL` 提高到 `500cm/s²`，`35m/s` 也主要适用于直线飞行或非常缓慢的大半径转弯。

不同偏航角速度下，保持 `35m/s` 所需的向心加速度如下：

| 偏航角速度 | 所需向心加速度 |
|---:|---:|
| 2°/s | 122cm/s² |
| 5°/s | 305cm/s² |
| 10°/s | 611cm/s² |
| 20°/s | 1222cm/s² |
| 25°/s | 1527cm/s² |

不建议为了在急转弯中维持 `35m/s` 而关闭 `FOLLOW_EXT_ENABLE_TURN_ACCEL_LIMIT`。关闭后，速度和偏航角速度组合可能超出飞行器能力，造成转弯严重外扩、倾角饱和、掉高和速度控制失效。

### 11.6 转弯加速度预算比例

推荐保留：

```cpp
static constexpr float FOLLOW_EXT_TURN_ACCEL_BUDGET_RATIO = 0.6f;
```

在日志确认动力、倾角和高度控制仍有充足余量后，可以小幅提高到：

```text
0.65～0.70
```

不建议接近 `1.0`，否则几乎没有水平加速度留给：

- 抗风。
- 速度误差修正。
- 避障和围栏处理。
- 视觉指令变化。
- 导航估计扰动。

如果需要显著提高转弯时的速度，应优先评估飞行器真实水平加速度和倾角能力，而不是单纯提高预算比例。

### 11.7 倾角和动力限制

需要检查以下参数：

```text
PSC_ANGLE_MAX
ANGLE_MAX
```

当 `PSC_ANGLE_MAX = 0` 时，位置控制器使用全局 `ANGLE_MAX`。高速稳定飞行所需的俯仰角由机体空气阻力、质量、动力系统和目标速度共同决定，不能只根据速度数值直接计算出通用参数。

不要为了追求 `35m/s` 直接提高倾角限制，应先检查飞行日志：

- 期望俯仰角是否持续达到限制值。
- 实际速度是否持续低于目标速度。
- 油门是否长时间接近饱和。
- 高度是否在高速前飞或转弯时下降。
- 电池电压、电流和电调温度是否异常。
- 速度控制器积分项是否持续增大。

如果倾角或油门已经持续饱和，继续提高 `FOLE_SPEED` 或 `WPNAV_SPEED` 不会提高实际速度，只会增大速度误差和控制器积分饱和风险。

`PSC_VELXY_P`、`PSC_VELXY_I`、`PSC_VELXY_D` 和 `PSC_VELXY_FF` 可能影响高速速度跟踪，但不应在没有日志分析和标准位置控制调参的情况下直接修改。

### 11.8 当前10Hz控制更新的高速风险

当前视觉模式使用 `ten_hz_flag`，每约 `100ms` 更新一次视觉误差、速度目标和机体系到NE坐标系的转换。

在 `35m/s` 时，每个控制更新周期飞行距离为：

\[
35\times0.1=3.5m
\]

这意味着视觉误差、偏航角速度和前向速度方向在每次更新之间可能明显滞后。对于 `35m/s` 高速视觉跟踪，建议后续将控制结构拆分为：

```text
视觉输入和误差更新：至少20～30Hz
速度、加速度和坐标转换输出：约50Hz或飞控主循环频率
```

尤其应让机体系前向速度到NE坐标系的转换持续使用当前实时航向更新，而不是只在10Hz时转换一次。

### 11.9 500ms通信超时的高速风险

当前控制包超时常量为：

```cpp
CONTROL_PACKET_TIMEOUT_MS = 500;
```

在 `35m/s` 时，飞行器从最后一个有效数据包到检测出超时之前，理论上已经飞行：

\[
35\times0.5=17.5m
\]

检测到超时后，飞行器仍然需要较长距离才能制动到悬停。高速测试时，在通信链路稳定且数据发送频率足够高的前提下，建议考虑将超时降低到 `100～200ms`，并设计分级处理：

```text
短时丢包：立即开始平滑减速
持续丢包：制动到悬停或进入预定安全模式
严重异常：执行任务定义的失效保护动作
```

不能只缩短超时时间而不检查通信周期和抖动，否则正常的数据发送延迟也可能频繁触发失效保护。

### 11.10 分阶段测试建议

#### 第一阶段：低速验证

```text
FOLE_SPEED  = 1500cm/s
WPNAV_SPEED = 2000cm/s
WPNAV_ACCEL = 250cm/s²
```

验证误差方向、偏航控制、垂向控制、视觉减速和通信超时行为。

#### 第二阶段：中速验证

```text
FOLE_SPEED  = 2500cm/s
WPNAV_SPEED = 3000cm/s
WPNAV_ACCEL = 300～400cm/s²
```

检查速度跟踪误差、倾角、推力余量和转弯外扩。

#### 第三阶段：接近目标速度

```text
FOLE_SPEED  = 3500cm/s
WPNAV_SPEED = 4000cm/s
WPNAV_ACCEL = 400cm/s²
```

检查制动距离、通信异常和目标短时丢失行为。

#### 第四阶段：35m/s以上目标

```text
FOLE_SPEED  = 4500cm/s
WPNAV_SPEED = 4500～5000cm/s
WPNAV_ACCEL = 400～500cm/s²
```

仅在前面阶段均有足够安全余量、测试空域足够大、定位和通信稳定、动力系统经过验证后进入该阶段。

### 11.11 高速配置结论

如果目标是“目标居中、直线飞行时超过 `35m/s`”，可以从以下配置逐级测试：

```text
FOLE_SPEED  = 4500cm/s
WPNAV_SPEED = 4500～5000cm/s
WPNAV_ACCEL = 400～500cm/s²
```

如果要求急转弯时仍始终超过 `35m/s`，则所需向心加速度、倾角和动力会快速增加，当前多旋翼控制能力和安全策略通常不适合通过参数强行实现。正常情况下应保留转向加速度限速，让飞行器在转向时主动降低前速。
