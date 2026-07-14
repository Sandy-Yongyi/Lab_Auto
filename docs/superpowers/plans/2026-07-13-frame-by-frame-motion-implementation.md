# 按帧运动功能实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**目标：** 将启动策略改为整数配置，完成三台设备共用的按帧状态机、静态全局 X 计算、动态逐枪 X 插补、模式化设备参数界面及安全回归测试。

**架构：** 启动层只负责把 `ModeConfig.toml` 的整数映射成现有策略字符串；界面层根据启动时缓存的策略选择字段；运动层把帧几何搜索和 X 数学计算放入独立纯逻辑辅助模块，`MotionOutFxFramePlanning` 只负责每台设备的单向状态机和轴指令。三台设备仍通过 `MachineAxisMap` 写入现有 48 轴报文。

**技术栈：** Python 3、`unittest`、wxPython、TOML、dataclasses、现有 PLC `AxisData`/`MachineAxisMap`。

## 全局约束

- 不删除或改写与本功能无关的原有注释。
- 所有 `__init__.py` 必须保持空文件，不写入任何代码。
- 不修改用户当前变更的 `model/tomls/PlcConfig.toml`。
- 不修改或重新引入对 `model/motionplan/MotionOutLiftPlanning.py` 的依赖；只复用其已确认的慢进慢退数学规则。
- 保持 `AXIS_LIST_COUNT = 48` 和现有 SN0/SN1/SN2 轴序号。
- 按帧规划器不能导入完整工件数据结构或完整工件运动规划器。
- 所有生产代码必须先有失败测试，再做最小实现。
- 配置非法、点云缺失或单枪无有效 X 时，关闭对应 X 的喷涂状态，不发送主动喷涂运动。

---

### 任务 1：整数启动策略与重启后生效

**文件：**
- 修改：`model/utils/StrategyUtil.py`
- 修改：`model/tomls/ModeConfig.toml`
- 修改：`control/MainFrameControl.py`
- 新建：`tests/__init__.py`
- 新建：`tests/test_strategy_mode_config.py`

**接口：**
- 输入：`strategy_name` 的 TOML 整数值。
- 输出：`strategy_name_from_code(value: object) -> str`，返回现有内部策略字符串。
- 下游：`MainFrameController.strategy_name` 在窗口构造时确定，启动按钮和设备参数窗口只使用缓存值。

- [x] **步骤 1：编写整数映射失败测试**

先新建空的 `tests/__init__.py`，确保所有专项测试都可以通过模块路径稳定运行。

```python
import unittest

from model.utils.StrategyUtil import strategy_name_from_code


class StrategyModeConfigTests(unittest.TestCase):
    def test_maps_supported_integer_codes(self):
        self.assertEqual(strategy_name_from_code(1), "frame_by_frame")
        self.assertEqual(strategy_name_from_code(2), "complete_workpiece")
        self.assertEqual(strategy_name_from_code(3), "continuous_bidirectional")

    def test_rejects_bool_string_and_unknown_code(self):
        for value in (True, "1", 0, 4, None):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    strategy_name_from_code(value)
```

- [x] **步骤 2：运行测试确认失败原因正确**

运行：`python -m unittest tests.test_strategy_mode_config -v`

预期：导入失败，提示 `strategy_name_from_code` 尚不存在。

- [x] **步骤 3：实现整数到内部策略的唯一映射**

在 `model/utils/StrategyUtil.py` 增加：

```python
STRATEGY_CODE_MAP = {
    1: FRAME_QUEUE_STRATEGY,
    2: COMPLETE_WORKPIECE_STRATEGY,
    3: CONTINUOUS_BIDIRECTIONAL_STRATEGY,
}


def strategy_name_from_code(value: object) -> str:
    """把 ModeConfig.toml 的整数策略映射为内部策略名称。"""
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"strategy_name 必须是整数 1/2/3，当前值: {value!r}")
    try:
        return STRATEGY_CODE_MAP[value]
    except KeyError as exc:
        raise ValueError(f"strategy_name 只支持 1/2/3，当前值: {value}") from exc
```

把 `ModeConfig.toml` 增加为：

```toml
# 数据采集和运动策略：1=frame_by_frame，2=complete_workpiece，3=continuous_bidirectional
strategy_name = 1
```

在 `MainFrameController.__init__` 加载 `mode_config` 后立即缓存：

```python
from model.utils.StrategyUtil import (
    is_complete_workpiece_mode,
    strategy_name_from_code,
)

self.strategy_name = strategy_name_from_code(self.mode_config.get("strategy_name"))
```

把 `on_start` 中的硬编码替换为：

```python
strategy_name = self.strategy_name
```

不得在 `on_start` 重新解析 `strategy_name`，确保修改配置后必须重启软件。

- [x] **步骤 4：增加主界面硬编码回归测试**

在同一测试文件增加：

```python
from pathlib import Path


def test_main_frame_uses_cached_strategy(self):
    source = Path("control/MainFrameControl.py").read_text(encoding="utf-8")
    self.assertIn("self.strategy_name = strategy_name_from_code", source)
    self.assertIn("strategy_name = self.strategy_name", source)
    self.assertNotIn('validate_strategy_name("complete_workpiece")', source)
```

- [x] **步骤 5：运行任务测试并提交**

运行：`python -m unittest tests.test_strategy_mode_config -v`

预期：全部通过。

```powershell
git add model/utils/StrategyUtil.py model/tomls/ModeConfig.toml control/MainFrameControl.py tests/__init__.py tests/test_strategy_mode_config.py
git commit -m "feat: load motion strategy from integer config"
```

---

### 任务 2：按模式显示参数并校验每台设备 Y 范围

**文件：**
- 删除：`model/tomls/MachineConfig.toml`
- 新建：`model/tomls/MachineConfig1.toml`
- 新建：`model/tomls/MachineConfig2.toml`
- 新建：`model/tomls/MachineConfig3.toml`
- 新建：`model/utils/MachineConfigUtil.py`
- 修改：`view/MachineConfigFrame.py`
- 修改：`control/MachineConfigFrameControl.py`
- 修改：`control/MainFrameControl.py`
- 修改：`control/PlcCommunicationProcess.py`
- 修改：`model/dataprocess/complete_workpiece/GunDistributor.py`
- 修改：`model/motionplan/MachineAxisMap.py`
- 新建：`tests/test_machine_config_selection.py`
- 新建：`tests/test_machine_axis_limits.py`

**接口：**
- 输入：启动时缓存的内部 `strategy_name`、设备 SN 和设备配置。
- 输出：`MachineConfigFrame(..., strategy_name: str)`；按帧模式固定 17 个字段；完整工件保持现有字段和分页。
- 运行时：`tracking`、`y_move_min`、`y_move_max` 可以通过现有控制队列更新。
- 配置文件：按帧使用 `MachineConfig1.toml`，完整工件使用 `MachineConfig2.toml`，连续双向使用 `MachineConfig3.toml`。
- PLC 出口：所有轴命令写入 `AxisList` 前，按实际轴类型限制位置和速度。

- [x] **步骤 1：编写字段选择、配置路径和参数校验失败测试**

测试覆盖策略到三个文件名的映射、三个 TOML 文件可解析、按帧字段顺序、`tracking` 和 Y 范围校验，以及三台设备全部 48 根轴的 PLC 出口限位。

- [x] **步骤 2：运行测试确认失败**

运行：

```powershell
python -m unittest tests.test_machine_config_selection -v
python -m unittest tests.test_machine_axis_limits -v
```

预期：配置选择模块不存在，且越界轴命令未被限制。

- [x] **步骤 3：拆分三个策略配置并增加按帧运行时字段**

将原配置完整复制为 `MachineConfig2.toml` 和 `MachineConfig3.toml`；`MachineConfig1.toml` 在 `[0]`、`[1]`、`[2]` 中增加：

```toml
tracking = 0
y_move_min = 0
y_move_max = 430
```

当前三台设备 Y 限位均为 `[0, 430]`；如果后续设备限位不同，初始值必须随 `min_limit_pos/max_limit_pos` 的 Y 项调整。

在 `PlcCommunicationProcess.runtime_param_keys` 增加：

```python
"tracking", "y_move_min", "y_move_max",
```

- [x] **步骤 4：实现模式化字段选择**

在 `MachineConfigFrame` 中保留现有完整工件字段为 `COMPLETE_WORKPIECE_PARAM_KEYS`，新增：

```python
FRAME_BY_FRAME_PARAM_KEYS = [
    "tracking", "y_move_min", "y_move_max",
    "out_front_x_offset", "out_after_x_offset",
    "x_pos_speed", "x_recip_speed",
    "out_up_y_offset", "out_down_y_offset",
    "y_pos_speed", "y_recip_speed",
    "out_z_front_offset", "out_z_after_offset",
    "z_back_speed", "z_zeroing_speed",
    "x_status_offset", "outside_total_cycles",
]
```

增加中文标签，并修改构造函数和选择逻辑：

```python
def __init__(self, parent, sn: int, control_queue=None,
             strategy_name="frame_by_frame", title_prefix="设备参数设置"):
    self.strategy_name = validate_strategy_name(strategy_name)

def _get_param_keys(self, sn: int) -> list[str]:
    if self.strategy_name == "frame_by_frame":
        return self.FRAME_BY_FRAME_PARAM_KEYS
    return self.COMPLETE_WORKPIECE_PARAM_KEYS.get(sn, [])

def _uses_dual_config_pages(self):
    return (
        self.strategy_name == "complete_workpiece"
        and self.machine_cfg.get("type") == "xn_side"
    )
```

`MainFrameController.open_machine_config` 必须传入缓存模式：

```python
dlg = MachineConfigFrame(
    self,
    sn,
    self.control_queue,
    strategy_name=self.strategy_name,
)
```

- [x] **步骤 5：实现设备参数限位校验**

在 `_get_param_range_rules` 中通过 `get_axis_position_limits(machine_cfg, "y")` 生成：

```python
"tracking": (0, 1),
"y_move_min": (y_pos_min, y_pos_max),
"y_move_max": (y_pos_min, y_pos_max),
```

在 `_validate_params` 的普通范围校验后增加：

```python
if "y_move_min" in values and "y_move_max" in values:
    if int(values["y_move_min"]) >= int(values["y_move_max"]):
        raise ValueError("y_move_min 必须小于 y_move_max")
```

- [x] **步骤 6：在 PLC 写轴统一出口限制全部轴的位置和速度**

`MachineAxisMap.apply_device_axes_to_list` 在写入每根实际轴前读取 `min_limit_pos`、`max_limit_pos` 和 `max_limit_speed`。逻辑 Y 广播到 `y1~y6` 时，对广播后的每根实际轴分别限制。

- [x] **步骤 7：运行任务测试**

运行：

```powershell
python -m unittest tests.test_machine_config_selection -v
python -m unittest tests.test_machine_axis_limits -v
python -m unittest discover -s tests -v
```

预期：全部通过。

---

### 任务 3：动态 Z 窗口、连续边界帧和 X 范围搜索

**文件：**
- 修改：`model/motionplan/motionutil/FrameSearchHelper.py`
- 修改：`model/tomls/SprayConfig.toml`
- 新建：`tests/test_frame_motion_geometry.py`

**接口：**
- 输出：`FrameWindow(start: int, center: int, end: int)`。
- 输出：`build_window(machine_cfg, runtime_cfg, z_cur, frame_count) -> FrameWindow`。
- 输出：`has_start_signature(frames, window, count) -> bool`、`has_end_signature(...) -> bool`、`window_is_empty(...) -> bool`。
- 输出：`collect_x_range(frames, window, y_min, y_max) -> tuple[int | None, int | None]`。

- [x] **步骤 1：编写几何和边界失败测试**

```python
import unittest

from model.formats.frame_by_frame.AxisFrameDataFormat import AxisData, AxisFrameData
from model.motionplan.motionutil.FrameSearchHelper import FrameSearchHelper


def populated_frame(y=1500, x_min=100, x_max=300):
    return AxisFrameData(FrameData=[AxisData(y, x_max, x_min)])


def empty_frame():
    return AxisFrameData(FrameData=[AxisData(0, 0, 0)])


class FrameMotionGeometryTests(unittest.TestCase):
    def setUp(self):
        self.helper = FrameSearchHelper(z_threshold=10)

    def test_window_includes_current_z_position(self):
        cfg = {"z_position": 150, "out_z_front_offset": 50, "out_z_after_offset": 50}
        window = self.helper.build_window(cfg, {}, z_cur=20, frame_count=100)
        self.assertEqual((window.start, window.center, window.end), (12, 17, 22))

    def test_start_and_end_require_exact_consecutive_boundary_frames(self):
        window = self.helper.create_window(10, 15, 20, frame_count=30)
        start_frames = [empty_frame() for _ in range(30)]
        start_frames[10:15] = [populated_frame() for _ in range(5)]
        self.assertTrue(self.helper.has_start_signature(start_frames, window, 5))
        start_frames[16] = populated_frame()
        self.assertFalse(self.helper.has_start_signature(start_frames, window, 5))

        end_frames = [empty_frame() for _ in range(30)]
        end_frames[16:21] = [populated_frame() for _ in range(5)]
        self.assertTrue(self.helper.has_end_signature(end_frames, window, 5))

    def test_collects_x_range_inside_y_band_across_full_z_window(self):
        frames = [empty_frame() for _ in range(30)]
        frames[12] = populated_frame(y=1400, x_min=500, x_max=700)
        frames[18] = populated_frame(y=1450, x_min=100, x_max=900)
        window = self.helper.create_window(10, 15, 20, frame_count=30)
        self.assertEqual(self.helper.collect_x_range(frames, window, 1300, 1500), (100, 900))
```

- [x] **步骤 2：运行测试确认失败**

运行：`python -m unittest tests.test_frame_motion_geometry -v`

预期：`build_window`、`create_window` 或边界方法不存在。

- [x] **步骤 3：增加帧窗口和值搜索实现**

在 `FrameSearchHelper.py` 增加：

```python
from dataclasses import dataclass


@dataclass(frozen=True)
class FrameWindow:
    start: int
    center: int
    end: int


def create_window(self, start: int, center: int, end: int, frame_count: int) -> FrameWindow:
    last = max(0, frame_count - 1)
    start = max(0, min(int(start), last))
    center = max(start, min(int(center), last))
    end = max(center, min(int(end), last))
    return FrameWindow(start, center, end)


def build_window(self, machine_cfg, runtime_cfg, z_cur: int, frame_count: int) -> FrameWindow:
    z_position = int(machine_cfg.get("z_position", 0) or 0)
    front = int(runtime_cfg.get("out_z_front_offset", machine_cfg.get("out_z_front_offset", 0)) or 0)
    after = int(runtime_cfg.get("out_z_after_offset", machine_cfg.get("out_z_after_offset", 0)) or 0)
    center_z = z_position + int(z_cur or 0)
    return self.create_window(
        int((center_z - front) / self.z_threshold),
        int(center_z / self.z_threshold),
        int((center_z + after) / self.z_threshold),
        frame_count,
    )


def has_start_signature(self, frames, window: FrameWindow, count: int) -> bool:
    count = int(count)
    window_length = window.end - window.start + 1
    boundary_end = window.start + count
    if count <= 0 or count >= window_length:
        return False
    return (
        all(self.frame_has_data(self.get_frame_by_index(frames, idx)) for idx in range(window.start, boundary_end))
        and all(not self.frame_has_data(self.get_frame_by_index(frames, idx)) for idx in range(boundary_end, window.end + 1))
    )


def has_end_signature(self, frames, window: FrameWindow, count: int) -> bool:
    count = int(count)
    window_length = window.end - window.start + 1
    boundary_start = window.end - count + 1
    if count <= 0 or count >= window_length:
        return False
    return (
        all(not self.frame_has_data(self.get_frame_by_index(frames, idx)) for idx in range(window.start, boundary_start))
        and all(self.frame_has_data(self.get_frame_by_index(frames, idx)) for idx in range(boundary_start, window.end + 1))
    )


def window_is_empty(self, frames, window: FrameWindow) -> bool:
    return all(not self.frame_has_data(frames[idx]) for idx in range(window.start, window.end + 1))


def collect_x_range(self, frames, window: FrameWindow, y_min: int, y_max: int):
    values = self.collect_x_values(frames, window.start, window.end, y_min, y_max)
    return (min(values), max(values)) if values else (None, None)
```

- [x] **步骤 4：增加 SprayConfig 参数**

```toml
# Z窗口边界连续多少帧有数据时判定工件开始或结束
stage_detect_frame_count = 5

# 按帧X轴计算模式：0=静态全局范围，1=动态逐枪插补
frame_x_interpolation_enabled = 1
```

- [x] **步骤 5：运行测试**

运行：`python -m unittest tests.test_frame_motion_geometry -v`

预期：全部通过。

---

### 任务 4：静态全局 X、动态 X 最小值、慢进慢退和插补速度

**文件：**
- 新建：`model/motionplan/motionutil/FrameXMotionHelper.py`
- 新建：`tests/test_frame_x_motion_helper.py`

**接口：**
- 输出：`build_static_search_y_range(origin_pos, y_move_min, y_move_max, out_down_y_offset, out_up_y_offset) -> tuple[int, int]`。
- 输出：`build_dynamic_search_y_range(origin_pos, y_cur, out_down_y_offset, out_up_y_offset) -> tuple[int, int]`。
- 输出：`aggregate_static_x_range(x_ranges) -> tuple[int | None, int | None]`。
- 输出：`calculate_interpolation_speed(previous_y, current_y, previous_target, current_target, y_speed, max_speed, initial_speed) -> int`。
- 输出：`resolve_slow_offset(start_z_chain, end_z_chain, center_z, front_offset, after_offset, max_x_offset) -> int`。
- 输出：`build_final_x_target(base_x_min, x_position, current_x_offset, x_min_limit, x_max_limit) -> int`。

- [x] **步骤 1：编写纯数学失败测试**

```python
import unittest

from model.motionplan.motionutil.FrameXMotionHelper import FrameXMotionHelper


class FrameXMotionHelperTests(unittest.TestCase):
    def setUp(self):
        self.helper = FrameXMotionHelper()

    def test_interpolation_speed_uses_final_target_delta(self):
        speed = self.helper.calculate_interpolation_speed(
            previous_y=0,
            current_y=100,
            previous_target=500,
            current_target=100,
            y_speed=100,
            max_speed=500,
            initial_speed=200,
        )
        self.assertEqual(speed, 400)

    def test_interpolation_speed_is_capped(self):
        speed = self.helper.calculate_interpolation_speed(0, 10, 500, 100, 100, 300, 200)
        self.assertEqual(speed, 300)

    def test_final_target_subtracts_position_and_slow_offset(self):
        self.assertEqual(
            self.helper.build_final_x_target(800, 200, 100, 0, 1000),
            500,
        )

    def test_slow_offset_follows_front_middle_after_profile(self):
        self.assertEqual(self.helper.resolve_slow_offset(125, 100, 150, 50, 50, 200), 100)
        self.assertEqual(self.helper.resolve_slow_offset(160, 140, 150, 50, 50, 200), 200)
        self.assertEqual(self.helper.resolve_slow_offset(160, 175, 150, 50, 50, 200), 100)
```

- [x] **步骤 2：运行测试确认失败**

运行：`python -m unittest tests.test_frame_x_motion_helper -v`

预期：模块不存在。

- [x] **步骤 3：实现静态/动态搜索区间、全局范围、插补速度和最终目标**

```python
from model.motionplan.motionutil.AxisLimits import clamp_speed, clamp_to_limit_yx


class FrameXMotionHelper:
    @staticmethod
    def calculate_interpolation_speed(previous_y, current_y, previous_target,
                                      current_target, y_speed, max_speed,
                                      initial_speed):
        if previous_y is None or previous_target is None:
            return clamp_speed(int(initial_speed or 0), int(max_speed or 0))
        y_distance = abs(int(current_y) - int(previous_y))
        if y_distance == 0:
            return 0
        x_distance = abs(int(current_target) - int(previous_target))
        speed = round(x_distance * abs(int(y_speed or 0)) / y_distance)
        return clamp_speed(speed, int(max_speed or 0))

    @staticmethod
    def build_final_x_target(base_x_min, x_position, current_x_offset,
                             x_min_limit, x_max_limit):
        target = int(base_x_min) - int(x_position or 0) - int(current_x_offset or 0)
        return clamp_to_limit_yx(target, int(x_min_limit), int(x_max_limit))
```

- [x] **步骤 4：提取原慢进慢退数学规则**

在同一类实现 `resolve_slow_offset`，保持已确认的三段规则，不导入 `MotionOutLiftPlanning`：

```python
@staticmethod
def resolve_slow_offset(start_z_chain, end_z_chain, center_z,
                        front_offset, after_offset, max_x_offset):
    max_x_offset = max(0, int(max_x_offset or 0))
    start_z_chain = int(start_z_chain)
    end_z_chain = int(end_z_chain)
    center_z = int(center_z)
    if start_z_chain < center_z:
        if int(front_offset or 0) <= 0:
            return max_x_offset
        remaining = center_z - start_z_chain
        value = max_x_offset - max_x_offset / int(front_offset) * remaining
        return int(max(0, min(max_x_offset, value)))
    if end_z_chain < center_z:
        return max_x_offset
    if int(after_offset or 0) <= 0:
        return 0
    passed = end_z_chain - center_z
    value = max_x_offset - max_x_offset / int(after_offset) * passed
    return int(max(0, min(max_x_offset, value)))
```

- [x] **步骤 5：运行测试**

运行：`python -m unittest tests.test_frame_x_motion_helper -v`

预期：全部通过。

---

### 任务 5：实现单设备按帧状态机和轴指令

**文件：**
- 修改：`model/motionplan/MotionOutFxFramePlanning.py`
- 新建：`tests/test_motion_out_fx_frame_planning.py`

**接口：**
- 类：`MotionOutFxFramePlanning`。
- 兼容别名：`MotionOutFxPlanning = MotionOutFxFramePlanning`，迁移完成后由集成任务移除旧导入。
- 主入口：`auto_out_fx_move(machine_cfg, runtime_cfg, plc_data, frame_queue_manager) -> tuple[dict, bool, bool]`。
- 状态：每个 SN 独立保存 `IDLE/START/MIDDLE/END/RETURN_SAFE`、Y 方向、开始/结束计数、动态插补记忆。

- [x] **步骤 1：编写状态机失败测试**

测试文件使用真实 `AxisFrameData`、`MovingFrameData.AxisData` 和最小假 PLC 数据，至少包含：

```python
import unittest
from types import SimpleNamespace

from model.motionplan.MotionOutFxFramePlanning import MotionOutFxFramePlanning
from model.plc.MovingFrameData import AxisData, create_axis_list


class MotionOutFxFramePlanningTests(unittest.TestCase):
    def setUp(self):
        self.planner = MotionOutFxFramePlanning(
            spray_cfg={
                "stage_detect_frame_count": 5,
                "frame_x_interpolation_enabled": 1,
                "side_2d_cycle_axis": "y",
                "spray_pos_tolerance": 30,
            },
            read_data_cfg={"z_threshold": 10, "y_threshold": 10},
        )
        self.plc = SimpleNamespace(
            AxisList=create_axis_list(),
            ChainSpeed=100,
            ChainStatus="moving_forward",
        )

    def test_state_does_not_reenter_start_after_start_completion(self):
        state = self.planner._get_state(0)
        state.stage = "middle"
        self.planner._transition_for_signatures(state, start=True, center=False, end=False, empty=False, tracking=1)
        self.assertEqual(state.stage, "middle")

    def test_x_interpolation_memory_is_isolated_by_sn(self):
        self.planner._get_state(0).interpolation_targets["x1"] = 100
        self.assertNotIn("x1", self.planner._get_state(1).interpolation_targets)
```

继续增加行为测试：

- 连续开始帧只能触发 `IDLE -> START`。
- 跟踪开始/结束按照 `outside_total_cycles` 和 `side_2d_cycle_axis` 退出。
- 不跟踪开始在中心帧有数据后进入中间，不跟踪结束在窗口清空后回安全位。
- 跟踪开始/结束按每枪静态 Y 区间查找后汇总全局 `x_min/x_max`。
- `frame_x_interpolation_enabled = 0` 时，跟踪中间目标为 `global_x_min - x_position - out_front_x_offset`。
- 配置为 `1` 时，跟踪中间和不跟踪阶段按每枪动态 Y 区间更新 X 最小值、慢进慢退偏移、最终目标和速度。
- SN1/SN2 只读取 `y1`，满足 `spray_pos_tolerance` 后六个 Y 同时反向，R 轴全部为 0。
- 单枪无 X 数据时该 X 保持当前位置且 `Status = 0`。

- [x] **步骤 2：运行测试确认失败**

运行：`python -m unittest tests.test_motion_out_fx_frame_planning -v`

预期：新类构造参数、状态接口或行为尚不存在。

- [x] **步骤 3：建立明确的每 SN 状态结构**

在规划文件中增加：

```python
from dataclasses import dataclass, field


@dataclass
class DeviceFrameMotionState:
    stage: str = "idle"
    y_phase: str = "to_max"
    x_phase: str = "to_max"
    start_cycles: int = 0
    end_cycles: int = 0
    interpolation_bins: dict[str, int] = field(default_factory=dict)
    interpolation_y: dict[str, int] = field(default_factory=dict)
    interpolation_targets: dict[str, int] = field(default_factory=dict)
    interpolation_speeds: dict[str, int] = field(default_factory=dict)


def _get_state(self, sn: int) -> DeviceFrameMotionState:
    return self._work_states.setdefault(int(sn), DeviceFrameMotionState())
```

构造函数允许测试注入配置，生产环境仍从 TOML 加载：

```python
def __init__(self, spray_cfg=None, read_data_cfg=None):
    self.spray_cfg = spray_cfg or TomlLoader.load(...)
    self.read_data_cfg = read_data_cfg or TomlLoader.load(...)
```

- [x] **步骤 4：实现单向阶段切换**

```python
def _transition_for_signatures(self, state, start, center, end, empty, tracking):
    if state.stage == "idle" and start:
        state.stage = "start"
    elif state.stage == "start" and not tracking and center:
        state.stage = "middle"
    elif state.stage == "middle" and end:
        state.stage = "end"
    elif state.stage == "end" and not tracking and empty:
        state.stage = "return_safe"
```

跟踪阶段的计次完成由 X/Y 往复方法显式设置 `middle` 或 `return_safe`；不得根据持续存在的开始边界重新进入 `start`。

- [x] **步骤 5：实现 Y 同步往复和计次**

逻辑 Y 指令统一输出为键 `"y"`，由现有 `MachineAxisMap.apply_to_axis_list` 广播到 `y1~y6`。当前位置读取规则：设备有 `y` 时读取 `y`，否则只读取 `y1`。

```python
def _get_logical_y_pos(self, machine_cfg, plc_data):
    axis_map = get_axis_map(machine_cfg["type"], machine_cfg.get("install_orietation", "left"))
    axis_name = "y" if "y" in axis_map else "y1"
    return self.motion_to_target._get_axis_current_pos(plc_data, axis_map[axis_name])
```

只有 `abs(y1_current - y_target) <= spray_pos_tolerance` 时切换 `y_phase`，并在 `side_2d_cycle_axis == "y"` 且完成 `min -> max -> min` 时增加当前阶段计数。

- [x] **步骤 6：实现静态和动态 X 指令**

静态搜索对每把枪使用：

```text
origin_pos[i] + y_move_min - out_down_y_offset
到
origin_pos[i] + y_move_max + out_up_y_offset
```

汇总所有枪的 X 结果后，跟踪开始/结束所有 X 使用同一组：

```text
x_min_target = global_x_min - x_position - out_front_x_offset
x_max_target = global_x_max - x_position - out_after_x_offset
```

动态搜索对第 `i` 把枪使用：

```text
origin_pos[i] + y_cur - out_down_y_offset
到
origin_pos[i] + y_cur + out_up_y_offset
```

从 `FrameXMotionHelper` 得到：

```text
base_x_min -> current_x_offset -> final_x_target -> interpolation_speed
```

不跟踪开始/中间/结束分别使用递增、完整、递减的 `current_x_offset`；跟踪中间使用完整 `out_front_x_offset`。

- [x] **步骤 7：实现 Z、R 和喷涂状态**

- 跟踪开始/结束：Z 目标为最大限位，速度为链条速度。
- 跟踪中间：Z 回安全位，速度为 `z_back_speed`。
- 不跟踪全部阶段：Z 目标为 0。
- SN1/SN2：`r1~r6` 目标为 0、状态为 0。
- `x_status_offset` 按 `z_threshold` 换算提前帧数，只改变 X `Status` 生效时机，不改变 X 目标。
- 链条停止、对应枪无有效 X 或状态不允许喷涂时，X `Status = 0`。

- [x] **步骤 8：运行状态机测试并提交**

运行：`python -m unittest tests.test_motion_out_fx_frame_planning -v`

预期：全部通过。

```powershell
git add model/motionplan/MotionOutFxFramePlanning.py tests/test_motion_out_fx_frame_planning.py
git commit -m "feat: implement frame motion state machine"
```

---

### 任务 6：三设备集成、依赖收敛和全量验证

**文件：**
- 修改：`model/motionplan/MotionFrameByFramePlanning.py`
- 验证：`model/motionplan/MachineAxisMap.py`（现有逻辑 Y 广播应直接复用，不纳入修改和提交）
- 新建：`tests/test_motion_frame_by_frame_integration.py`

**接口：**
- 三台自动设备无论 `machine_cfg["type"]` 都调用同一个 `MotionOutFxFramePlanning`。
- 设备关闭、报警、激光异常、原始数据超时和手动模式的现有安全分支保持不变。

- [x] **步骤 1：编写三设备调用和轴映射失败测试**

```python
import unittest
from pathlib import Path

from model.motionplan.MachineAxisMap import apply_device_axes_to_list
from model.plc.MovingFrameData import AxisData, create_axis_list


class FrameByFrameIntegrationTests(unittest.TestCase):
    def test_auto_path_has_no_machine_type_gate(self):
        source = Path("model/motionplan/MotionFrameByFramePlanning.py").read_text(encoding="utf-8")
        self.assertNotIn('if machine_type == "out_fx"', source)
        self.assertIn("self.out_fx_planner.auto_out_fx_move", source)

    def test_logical_y_broadcasts_to_all_xn_side_y_axes(self):
        config = {
            "1": {"type": "xn_side", "install_orietation": "left"},
        }
        axis_list = create_axis_list()
        cmd = AxisData(Pos=123, Speed=50, Status=0)
        apply_device_axes_to_list(config, 1, {"y": cmd}, axis_list)
        for idx in (11, 14, 17, 20, 23, 26):
            self.assertIs(axis_list[idx], cmd)
```

继续增加集成测试，构造三个设备配置并验证：

- SN0 只写入索引 0~9。
- SN1 只写入索引 10~28。
- SN2 只写入索引 29~47。
- 一台设备的状态变化不修改其他设备轴命令。

- [x] **步骤 2：运行集成测试确认失败**

运行：`python -m unittest tests.test_motion_frame_by_frame_integration -v`

预期：仍存在 `machine_type == "out_fx"` 自动模式分支。

- [x] **步骤 3：统一三台设备调用**

在 `MotionFrameByFramePlanning` 中改为：

```python
from model.motionplan.MotionOutFxFramePlanning import MotionOutFxFramePlanning


def __init__(self):
    self.out_fx_planner = MotionOutFxFramePlanning()
```

自动设备启用分支不再读取 `machine_type`：

```python
axis_cmds, _, device_stop_chain = self.out_fx_planner.auto_out_fx_move(
    machine_cfg=machine_cfg,
    runtime_cfg=runtime_cfg,
    plc_data=proc.plc_data,
    frame_queue_manager=proc.frame_queue_manager,
)
```

设备禁用、回原点、手动模式和故障分支保持原逻辑及原注释。

- [x] **步骤 4：运行专项和全量测试**

运行：

```powershell
python -m unittest tests.test_strategy_mode_config -v
python -m unittest tests.test_machine_config_mode_ui -v
python -m unittest tests.test_frame_motion_geometry -v
python -m unittest tests.test_frame_x_motion_helper -v
python -m unittest tests.test_motion_out_fx_frame_planning -v
python -m unittest tests.test_motion_frame_by_frame_integration -v
python -m unittest discover -s tests -v
```

预期：全部测试通过，无异常堆栈。

- [x] **步骤 5：执行编译和依赖检查**

运行：

```powershell
python -m py_compile model/utils/StrategyUtil.py control/MainFrameControl.py view/MachineConfigFrame.py control/MachineConfigFrameControl.py control/PlcCommunicationProcess.py model/motionplan/motionutil/FrameSearchHelper.py model/motionplan/motionutil/FrameXMotionHelper.py model/motionplan/MotionOutFxFramePlanning.py model/motionplan/MotionFrameByFramePlanning.py
rg -n "MotionCompleteWorkpiecePlanning|MotionXNSidePlanning|MotionOutLiftPlanning" model/motionplan/MotionOutFxFramePlanning.py model/motionplan/MotionFrameByFramePlanning.py
```

预期：`py_compile` 返回 0；`rg` 在两个按帧文件中没有找到完整工件或已删除规划器依赖。

- [x] **步骤 6：检查工作区边界（未创建 Git 提交）**

运行：

```powershell
git status --short
git diff --check
git diff -- model/tomls/PlcConfig.toml
```

预期：`PlcConfig.toml` 仍只有用户原有修改；本功能未改动该文件，也未改动用户现有 `MotionOutLiftPlanning.py`。

```powershell
git add model/motionplan/MotionFrameByFramePlanning.py tests/test_motion_frame_by_frame_integration.py docs/superpowers/plans/2026-07-13-frame-by-frame-motion-implementation.md
git commit -m "feat: integrate frame motion for three devices"
```
