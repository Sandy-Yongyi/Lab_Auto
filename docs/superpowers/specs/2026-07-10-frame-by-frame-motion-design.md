# 按帧运动功能设计文档

## 目标

将采集和运动策略改为通过 `ModeConfig.toml` 选择，完成三台设备的按帧运动功能，并根据软件启动时选定的模式显示对应的设备参数。

本阶段只实现 `frame_by_frame`。现有 `complete_workpiece` 路径继续保留，但尚未完成的 `out_fx` 完整工件运动逻辑不属于本阶段范围。

## 项目约束

- 软件模式只在主程序启动时读取一次。软件运行期间修改模式配置，不改变当前生效模式。
- 除非原注释因本次修改而不再正确，否则必须保留现有源码注释。
- 三台设备继续使用当前 48 轴 PLC 布局。
- 三台设备统一调用同一个按帧运动规划入口。
- 按帧运动规划器不能依赖完整工件模式的数据结构。

## 配置约定

### ModeConfig.toml

增加以下顶层配置：

```toml
# 数据采集和运动策略：1=frame_by_frame，2=complete_workpiece，3=continuous_bidirectional。
# 修改后需要重启软件才能生效。
strategy_name = 1
```

配置值与内部策略名称的映射为：`1 -> frame_by_frame`、`2 -> complete_workpiece`、`3 -> continuous_bidirectional`。本项目在配置文件中明确默认为 `1`。如果配置了不支持的整数，软件必须显示错误并停止启动，不能静默切换到其他策略。

`MainFrameController` 在构造时读取并校验整数配置，将它转换为现有内部策略字符串，然后把同一个内部策略传给参数窗口和所有子进程。点击启动按钮时不能重新读取或切换策略，从而保证修改配置后必须重启软件才能生效。

### MachineConfig1/2/3.toml

设备配置按启动策略拆分：

- `frame_by_frame` 使用 `MachineConfig1.toml`。
- `complete_workpiece` 使用 `MachineConfig2.toml`。
- `continuous_bidirectional` 使用 `MachineConfig3.toml`。

主界面设备参数窗口和 PLC 子进程必须使用启动时已经缓存的同一个策略选择配置文件，不能再硬编码旧的 `MachineConfig.toml`。`MachineConfig2.toml` 和 `MachineConfig3.toml` 初始保留拆分前的完整配置。

`MachineConfig1.toml` 的 SN0、SN1 和 SN2 都增加以下配置：

```toml
tracking = 0
y_move_min = 0
y_move_max = 430
```

`tracking` 只允许填写 `0` 或 `1`，默认值为 `0`，表示不跟踪。`y_move_min` 和 `y_move_max` 是设备坐标中的 Y 轴往复目标。每台设备的初始默认值分别取该设备配置的 Y 轴最小限位和最大限位，不能使用超过设备 Y 轴限位的固定默认值。

界面保存时必须满足：

```text
Y轴最小限位 <= y_move_min < y_move_max <= Y轴最大限位
```

每台设备继续使用各自的 `outside_total_cycles`。在按帧模式中，该值控制跟踪模式开始阶段和结束阶段需要完成的往复次数。

按帧运行时参数更新白名单需要在现有参数基础上增加 `tracking`、`y_move_min` 和 `y_move_max`。

### SprayConfig.toml

增加：

```toml
# 在 Z 窗口边界连续多少帧有数据时，判定为工件开始或结束。
stage_detect_frame_count = 5

# 按帧X轴计算模式：0=静态全局范围，1=动态逐枪插补。
frame_x_interpolation_enabled = 1
```

复用现有 `side_2d_cycle_axis`，选择往复计次轴：

- `"x"`：`x_min -> x_max -> x_min` 算一次往复。
- `"y"`：`y_move_min -> y_move_max -> y_move_min` 算一次往复。

边界检测帧数必须大于 0，并且必须小于每台设备配置的 Z 窗口所包含的帧数。检测帧数等于整个窗口长度时没有“其余空帧”，不能判定为开始或结束。

`frame_x_interpolation_enabled` 只允许为 `0` 或 `1`：

- `0`：跟踪和不跟踪的所有阶段都使用静态全局 X 范围。
- `1`：跟踪开始/结束仍使用静态全局 X 范围；跟踪中间及全部不跟踪阶段使用动态逐枪插补。

## 按模式显示设备参数窗口

设备参数窗口接收软件启动时已经缓存的策略，不再自行从配置文件重新读取当前生效策略。

在 `frame_by_frame` 模式下，三台设备都只显示一个参数页面，页面只包含以下参数：

1. `tracking`
2. `y_move_min`
3. `y_move_max`
4. `out_front_x_offset`
5. `out_after_x_offset`
6. `x_pos_speed`
7. `x_recip_speed`
8. `out_up_y_offset`
9. `out_down_y_offset`
10. `y_pos_speed`
11. `y_recip_speed`
12. `out_z_front_offset`
13. `out_z_after_offset`
14. `z_back_speed`
15. `z_zeroing_speed`
16. `x_status_offset`
17. `outside_total_cycles`

在 `complete_workpiece` 模式下，保留当前各设备的参数列表，并保留 `xn_side` 设备的“柜体配置/平板配置”分页。

界面必须校验 `tracking` 只能为 `0` 或 `1`，校验 `y_move_min/y_move_max` 满足当前设备 Y 轴限位和大小关系，并继续按照现有轴限位和 `SprayConfig.toml` 参数范围校验速度、偏移量和往复次数。

## 运动规划调用

`MotionFrameByFramePlanning` 对 SN0、SN1 和 SN2 都调用 `MotionOutFxFramePlanning.auto_out_fx_move()`，不再根据 `machine_cfg["type"]` 分支选择其他运动规划器。

运动规划器返回逻辑轴指令，最终通过 `MachineAxisMap` 写入实际 48 轴位置：

- SN0：`z`、`y`、`x1` 到 `x8`。
- SN1/SN2：`z`、`y1/x1/r1` 到 `y6/x6/r6`。

对于 SN1/SN2，一个逻辑 Y 轴往复目标同步写入 `y1` 到 `y6`。往复方向只读取 `y1` 的当前位置判断，满足 `abs(y1_current - y_target) <= spray_pos_tolerance` 后，`y1` 到 `y6` 同时反向。其他 Y 轴的到位状态不参与反向判断。运动规划器明确把 `r1` 到 `r6` 的位置设置为 `0`，喷涂状态设置为 `0`。

每台设备按 SN 独立保存状态。一台设备的阶段切换或往复计数不能影响其他设备。

## Z 窗口和点云范围提取

每台设备计算以下闭区间帧索引：

```text
window_start = (z_position + z_cur - out_z_front_offset) / z_threshold
window_center = (z_position + z_cur) / z_threshold
window_end = (z_position + z_cur + out_z_after_offset) / z_threshold
```

`z_cur` 为当前设备 Z 轴的实际位置。三个窗口索引每个规划周期都必须使用最新 `z_cur` 重新计算。索引计算沿用现有整数帧规则，并限制在实际帧堆栈范围内。每个运动规划周期只扫描一次完整的 `window_start..window_end` 范围。

扫描结果需要包含：

- 每一帧是否存在有效数据；
- `window_center` 之前、中点位置和中点之后的有效帧索引；
- 静态模式下，每把喷枪固定 Y 区间内的 X 最小值和最大值，以及汇总后的全局 X 最小值和最大值；
- 动态插补模式下，每把喷枪当前 Y 区间内的 X 最小值。

静态模式下，第 `i` 把喷枪的点云 Y 区间为：

```text
search_y_min[i] = origin_pos[i] + y_move_min - out_down_y_offset
search_y_max[i] = origin_pos[i] + y_move_max + out_up_y_offset
```

动态插补模式下，第 `i` 把喷枪按照当前逻辑 Y 轴位置查找：

```text
search_y_min[i] = origin_pos[i] + y_cur - out_down_y_offset
search_y_max[i] = origin_pos[i] + y_cur + out_up_y_offset
```

SN0 建立 8 个喷枪区间，SN1/SN2 建立 6 个喷枪区间。每把喷枪的 X 范围都必须在完整 Z 窗口中计算，不能只使用窗口前半段或后半段。

## 开始和结束边界判定

设 `N = stage_detect_frame_count`。

只有同时满足以下条件，才判定为工件开始：

- `window_start` 到 `window_start + N - 1` 的每一帧都存在数据；
- 从下一帧到 `window_end` 的其余所有帧都为空。

只有同时满足以下条件，才判定为工件结束：

- `window_end - N + 1` 到 `window_end` 的每一帧都存在数据；
- 从 `window_start` 到前一帧的其余所有帧都为空。

因此，配置为 5 帧时要求边界连续 5 帧都有数据，不是 5 帧范围内任意一帧有数据。

边界判定只能触发当前状态允许的切换。设备离开空闲状态后，后续检测到的开始条件必须忽略；设备进入中间状态前，检测到的结束条件也必须忽略。这样可以避免同一个持续存在的边界数据让设备重复进入开始阶段。

如果中点帧临时为空，但中点前后同时存在数据，运动规划器保持当前阶段，不因为点云局部缺口重新判断开始或结束。

## 单台设备状态机

每台设备使用以下状态：

```text
IDLE -> START -> MIDDLE -> END -> RETURN_SAFE -> IDLE
```

同一个工件的状态只能单向切换：

- `IDLE -> START`：检测到开始边界。
- 跟踪模式 `START -> MIDDLE`：`side_2d_cycle_axis` 选定的 X 轴或 Y 轴往复次数达到该设备的 `outside_total_cycles`。
- 不跟踪模式 `START -> MIDDLE`：有效数据到达 `window_center`。
- `MIDDLE -> END`：检测到结束边界。
- 跟踪模式 `END -> RETURN_SAFE`：`side_2d_cycle_axis` 选定的 X 轴或 Y 轴往复次数达到 `outside_total_cycles`。
- 不跟踪模式 `END -> RETURN_SAFE`：完整 Z 窗口中的数据全部离开。
- `RETURN_SAFE -> IDLE`：设备到达安全位置或原点指令目标，随后清空全部阶段状态和往复计数。

开始阶段和结束阶段使用相互独立的往复计数。进入中间阶段时清空开始阶段计数；进入结束阶段时新建一套结束阶段计数。

## 两种 X 计算模式

### 静态全局范围

先按照每把喷枪固定的 `origin_pos[i] + y_move_min - out_down_y_offset` 到 `origin_pos[i] + y_move_max + out_up_y_offset` 区间，在完整 Z 窗口内分别查找 X 最小值和最大值。随后汇总所有喷枪的有效结果，取全部结果中的全局 X 最小值和全局 X 最大值。

跟踪模式的开始和结束阶段始终使用静态全局范围，所有 X 轴共用同一组 `x_min/x_max`。当 `frame_x_interpolation_enabled = 0` 时，跟踪中间及全部不跟踪阶段也使用静态全局范围。

静态模式第一次定位使用界面配置的 `x_pos_speed`，X 往复使用 `x_recip_speed`。

### 动态逐枪插补

只有 `frame_x_interpolation_enabled = 1` 时启用，适用范围为跟踪中间阶段及全部不跟踪阶段。每把喷枪根据 `origin_pos[i] + y_cur - out_down_y_offset` 到 `origin_pos[i] + y_cur + out_up_y_offset` 的动态 Y 范围，在完整 Z 窗口内独立查找并更新 X 最小值。

动态插补使用两个相互独立的计算结果：

```text
base_x_min = 当前Y区间内重新查找到并完成点云方向转换、但尚未扣除x_position的X最小值
final_x_target = base_x_min - x_position - current_x_offset
```

动态插补会同时改变 `base_x_min`、`final_x_target` 和 X 轴速度，不能只改变速度而保持原 X 最小值。

`current_x_offset` 继续由原 `MotionOutLiftPlanning._resolve_cabinet_x_dis` 的慢退/慢进曲线计算：

- 不跟踪开始阶段：从 `0` 逐渐增加到 `out_front_x_offset`。
- 不跟踪中间阶段：保持为完整的 `out_front_x_offset`。
- 不跟踪结束阶段：从 `out_front_x_offset` 逐渐减小到 `0`。
- 跟踪中间阶段：使用完整的 `out_front_x_offset`。

因此 `0 <= current_x_offset <= out_front_x_offset`。`out_after_x_offset` 不参与动态 X 最小值的慢退/慢进计算，只用于跟踪开始/结束阶段静态全局 `x_max` 目标修正。

慢退/慢进偏移沿用原完整工件实现的整数转换规则：浮点计算完成后使用 `int` 截断，再限制到 `0..out_front_x_offset`，不能改成四舍五入。

按照 `y_threshold` 将 `y_cur` 划分为 Y 区间：

- 第一次进入使用动态插补的阶段时，因为没有上一 Y 区间，X 使用 `x_pos_speed` 定位到首个目标。
- 只有 Y 进入新区间时，才重新查找 X 最小值并计算新的 X 插补速度。
- Y 保持在同一区间时，继续使用上一 X 目标和速度，避免点云变化造成 X 抖动。

进入新 Y 区间后的 X 插补速度为：

```text
y_time = abs(y_current - y_previous) / y_recip_speed
x_interp_speed = abs(final_x_target_current - final_x_target_previous) / y_time

等价于：

x_interp_speed = abs(final_x_target_current - final_x_target_previous)
                 * y_recip_speed
                 / abs(y_current - y_previous)
```

计算结果必须限制在当前设备 X 轴最大速度以内。动态插补负责更新当前 Y 区间对应的 X 最小值；慢退、保持和慢进负责独立计算 `current_x_offset`。两部分共同得到 `final_x_target`，插补速度按照相邻 Y 区间对应的最终 X 目标差值计算。

## 跟踪运动（`tracking = 1`）

### 开始阶段

- 每把喷枪先在各自固定 Y 区间内查找 X 范围，再汇总得到同一组静态全局 X 范围。
- 所有 X 轴在经过坐标转换和偏移修正后的全局 `x_min` 与 `x_max` 之间同步往复。
- 所有 Y 轴在配置的 `y_move_min` 和 `y_move_max` 之间同步往复。
- Z 轴目标为该设备配置的最大限位，速度使用 PLC 当前链条速度。
- 使用 `side_2d_cycle_axis` 选择计次轴，使用 `outside_total_cycles` 判断开始阶段完成。

### 中间阶段

- 当 `frame_x_interpolation_enabled = 0` 时，所有 X 轴定位到 `静态全局 x_min - x_position - out_front_x_offset`。
- 当 `frame_x_interpolation_enabled = 1` 时，每个 X 轴根据当前 `y_cur` 对应的动态 Y 区间独立更新 X 最小值，减去完整 `out_front_x_offset` 得到最终目标，并使用自动计算的插补速度定位。
- 本阶段 X 轴不执行 `x_min/x_max` 往复。
- Y 轴继续在 `y_move_min` 和 `y_move_max` 之间往复，但不通过往复次数退出本阶段。
- Z 轴使用 `z_back_speed` 返回配置的安全位置。
- 一直保持中间阶段，直到检测到结束边界。

### 结束阶段

- 每把喷枪重新在各自固定 Y 区间内查找 X 范围，汇总后所有 X 轴使用同一组静态全局 X 范围并同步往复。
- Y 轴在 `y_move_min` 和 `y_move_max` 之间往复。
- Z 轴目标为最大限位，速度使用链条速度。
- 使用独立的新计数，并按照 `side_2d_cycle_axis` 选择的轴完成 `outside_total_cycles` 次往复。
- 往复完成后进入 `RETURN_SAFE`，随后结束当前工件。

## 不跟踪运动（`tracking = 0`）

开始、中间和结束三个阶段都把 Z 轴位置设置为 `0`，不执行跟随运动。Y 轴在三个阶段中始终在配置的 `y_move_min` 和 `y_move_max` 之间往复。SN1/SN2 的 R 轴始终保持在 `0`。

当 `frame_x_interpolation_enabled = 0` 时，所有 X 轴使用静态全局 X 目标。当配置为 `1` 时，每个 X 轴根据对应喷枪当前动态 Y 区间内的有效 X 数据独立计算目标和插补速度。

X 目标沿用原 `MotionOutLiftPlanning._build_cabinet_x_target` 和 `_resolve_cabinet_x_dis` 的慢进慢退位置变化方式：

- 开始阶段：先进入到每把喷枪对应的最小目标，然后随着工件到达中点逐渐退回到配置的偏移位置。
- 中间阶段：保持在完整偏移位置不变。
- 结束阶段：随着工件离开中点并接近后边界，从完整偏移位置逐渐进入。

无论是否启用动态逐枪插补，上述慢退、保持和慢进 X 偏移求解都必须保留。启用插补时，每个 Y 区间重新查找 X 最小值，再减去当前慢退/慢进偏移得到最终 X 目标，同时自动计算 X 速度。

开始阶段通过数据到达中点退出，不使用往复次数。结束阶段只有在完整 Z 窗口变为空后才退出。

## 开枪状态和 `x_status_offset`

`x_status_offset` 在按帧模式中必须生效，并继续作为每台设备可运行时更新的参数。

该值通过 `z_threshold` 换算成帧距离，并按照现有完整工件 `_has_front_outside_x_status_arrived` 和 `_has_after_outside_x_status_arrived` 的链条方向应用到工件到达边界。它只改变 X 轴喷涂 `Status` 的生效时机，不改变计算得到的 X 位置目标。

设备处于有效的开始、中间或结束喷涂阶段时，只有对应点云数据有效并且链条状态允许喷涂，X 轴才启用喷涂状态。如果某把喷枪的 Y 区间没有有效 X 数据，则该 X 轴保持当前位置并设置 `Status = 0`，不能使用其他喷枪的 X 范围代替。设备不处于有效工件阶段时，所有 X 轴喷涂状态均为 `0`。

## 安全和无效配置处理

- 帧堆栈不存在，或者设备处于空闲状态且没有有效数据：保持当前位置或使用现有逻辑返回安全位置/原点，所有喷涂状态关闭。
- 某把喷枪无法计算 X 范围：该 X 轴保持当前位置并关闭喷涂状态。
- `tracking`、`y_move_min/y_move_max`、`frame_x_interpolation_enabled`、计次轴或边界帧数配置无效：记录配置错误，关闭喷涂状态，不发送主动喷涂运动指令。
- 设备关闭、激光故障、伺服故障和原始数据超时继续使用现有 `MotionFrameByFramePlanning` 外层的回原点和停链处理。
- 所有规划器生成的轴目标和速度必须在写入 PLC `AxisList` 的统一出口再次校验。每根实际轴分别使用设备配置中的 `min_limit_pos`、`max_limit_pos` 和 `max_limit_speed`，逻辑 Y 广播到 `y1~y6` 后也必须逐轴校验。

## 测试方案

所有生产代码修改前先编写测试，测试范围包括：

1. 从 `ModeConfig.toml` 读取整数启动模式，包括 `1/2/3` 到内部策略字符串的映射、重启后才切换模式，以及非法值处理。
2. 按帧模式和完整工件模式的界面参数列表及分页行为。
3. `tracking`、`y_move_min/y_move_max`、`outside_total_cycles`、`stage_detect_frame_count` 和 `frame_x_interpolation_enabled` 的配置校验。
4. `z_cur` 改变后，开始、中点和结束三个 Z 窗口索引同步更新。
5. 连续帧开始边界和结束边界的精确判定。
6. 同一个工件只能单向切换状态，不能重复进入开始阶段。
7. 静态模式按各喷枪固定 Y 区间查找后汇总全局 `x_min/x_max`，所有 X 轴使用同一组目标。
8. 动态模式按 `y_cur` 和每把喷枪原点更新 X 最小值，减去当前慢退/慢进偏移得到最终 X 目标；首次使用 `x_pos_speed`，跨 Y 区间时按照最终目标差正确计算并限制插补速度，同一区间保持目标不抖动。
9. 跟踪模式开始、中间、结束三个阶段的轴目标，以及使用 X 轴或 Y 轴计次时的阶段完成条件。
10. 不跟踪模式到达中点和窗口清空时的状态切换，并验证慢退、保持和慢进 X 位置不被动态插补覆盖。
11. SN0 的 8 个 X 轴映射，以及 SN1/SN2 的 6 个 X 轴、6 个 Y 轴和 6 个 R 轴映射。
12. 三个策略选择正确的设备配置文件，并使用真实三设备配置验证全部 48 根轴的目标位置和速度在 PLC 出口被限制。
13. SN1/SN2 只使用 `y1` 和 `spray_pos_tolerance` 判断到位，并同步反转 `y1~y6`。
14. 单把喷枪缺少点云数据时的位置保持和喷涂状态隔离。
15. 按帧模式中 `x_status_offset` 的提前开枪时机。
16. 现有按帧/完整工件规划依赖测试和 Python 编译检查。
