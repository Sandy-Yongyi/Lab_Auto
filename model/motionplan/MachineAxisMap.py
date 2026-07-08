"""
设备轴索引映射配置模块

根据设备类型（MachineConfig.toml 的 type）和安装方向（install_orietation），
查找 PLC Axis_List 中对应的轴索引。

设备类型:
    - in_up:      内顶，axis_type = ["x"]
    - xn_side:    侧面云雀，axis_type = ["z", "y", "x1", "r1", ..., "x6", "r6"]
    - out_down:   外底，axis_type = ["y", "x"]
    - out_up:     外顶，axis_type = ["y"]
    - out_lift:   外侧二维往复机，axis_type = ["y"]

使用方式:
    from model.motionplan.MachineAxisMap import get_axis_map, get_axis_index

    # 获取完整轴映射
    axis_map = get_axis_map("xn_side", "right")
    # => {"z": 1, "y": 2, "x1": 3, "r1": 4, ...}

    # 获取单个轴索引
    z_idx = get_axis_index("xn_side", "right", "z")
    # => 1

    # 读取PLC数据
    z_pos = plc_data.AxisList[z_idx].Pos

    # 将运动结果写入PLC
    apply_to_axis_list(machine_data, "xn_side", "right", send_frame.AxisList)
"""

from model.utils.LoggerUtil import logger

# ============================================================
# PLC Axis_List 轴索引分配
#
# SendMovingFrameData.AxisList 共有 100 个 AxisData 槽位
# 以下为每种设备在各安装方向上的轴索引分配
#
# 当前 MachineConfig.toml 设备顺序如下:
#   sn=0 in_up    : 0      (x)
#   sn=1 xn_side  : 1~14   (z, y, x1, r1, x2, r2, x3, r3, x4, r4, x5, r5, x6, r6)
#   sn=2 out_down : 15~16  (y, x)
#   sn=3 out_up   : 17     (y)
#   sn=4 out_up   : 18     (y)
#   sn=5 out_lift : 19     (y)
#
# 修改轴索引时只需调整此字典，无需修改运动规划代码
# ============================================================

# ============================================================
# max_limit_speed / min_limit_pos / max_limit_pos 字段的轴对应关系
#
# MachineConfig.toml 中每种设备的 max_limit_speed / min_limit_pos / max_limit_pos
# 都按以下顺序对应轴:
#   in_up:    [x]
#   xn_side:  [z, y, x, r]   (所有 x1~x6 共用 x 配置，所有 r1~r6 共用 r 配置)
#   out_down: [y, x]
#   out_up:   [y]
#   out_lift: [y]
#
# 使用方式:
#   idx = AXIS_LIMIT_KEY["xn_side"]["z"]  # => 0
#   z_max = max_limit_pos[idx]
# ============================================================

AXIS_LIMIT_KEY: dict[str, dict[str, int]] = {
    # key: axis_name 或 "x"/"r"（所有同类轴的通用键）
    # value: 在 max_limit_speed / min_limit_pos / max_limit_pos 列表中的索引
    "in_up":      {"x": 0},
    "xn_side":    {"z": 0, "y": 1, "x": 2, "r": 3},
    "out_down":   {"y": 0, "x": 1},
    "out_up":     {"y": 0},
    "out_lift":   {"y": 0},
}


def get_axis_config_index(machine_type: str, axis_name: str) -> int | None:
    """获取轴在 max_limit_speed / min_limit_pos / max_limit_pos 中的索引。"""
    limit_key = AXIS_LIMIT_KEY.get(machine_type, {})
    if axis_name.startswith("x"):
        lookup = "x"
    elif axis_name.startswith("r"):
        lookup = "r"
    else:
        lookup = axis_name
    return limit_key.get(lookup)


def get_axis_config_value(machine_cfg: dict, axis_name: str, config_key: str, default: int = 0) -> int:
    """按设备通用轴定义读取配置值，如 safe_pos / max_limit_speed / min_limit_pos / max_limit_pos。"""
    machine_type = machine_cfg.get("type", "")
    idx = get_axis_config_index(machine_type, axis_name)
    values = machine_cfg.get(config_key, [])
    if idx is not None and isinstance(values, (list, tuple)) and idx < len(values):
        return int(values[idx] or 0)
    return default


def get_axis_speed_limit(machine_cfg: dict, axis_name: str, default: int = 300) -> int:
    """获取设备指定轴的最大速度。"""
    return get_axis_config_value(machine_cfg, axis_name, "max_limit_speed", default)


def get_axis_position_limits(machine_cfg: dict, axis_name: str, default_min: int = 0, default_max: int = 1000) -> tuple[int, int]:
    """获取设备指定轴的位置最小/最大限位。"""
    min_limit = get_axis_config_value(machine_cfg, axis_name, "min_limit_pos", default_min)
    max_limit = get_axis_config_value(machine_cfg, axis_name, "max_limit_pos", default_max)
    return min_limit, max_limit


def get_axis_safe_pos(machine_cfg: dict, axis_name: str, default: int = 0) -> int:
    """获取设备指定轴的安全位。"""
    return get_axis_config_value(machine_cfg, axis_name, "safe_pos", default)


MACHINE_AXIS_MAP = {
    # ---- 内顶 (sn=0) ----
    # axis_type: ["x"]
    "in_up": {
        "right": dict(x=0),
    },

    # ---- 侧面云雀 (sn=1) ----
    # axis_type: ["z", "y", "x1", "r1", ..., "x6", "r6"]
    "xn_side": {
        "right": dict(
            z=1,
            y=2,
            x1=3, r1=4,
            x2=5, r2=6,
            x3=7, r3=8,
            x4=9, r4=10,
            x5=11, r5=12,
            x6=13, r6=14,
        ),
    },

    # ---- 外底 (sn=2) ----
    # axis_type: ["y", "x"]
    "out_down": {
        "left": dict(y=15, x=16),
    },

    # ---- 外顶 (sn=3 右, sn=4 左) ----
    # axis_type: ["y"]
    "out_up": {
        "right": dict(y=17),
        "left": dict(y=18),
    },

    # ---- 外侧二维往复机 (sn=5 左) ----
    # axis_type: ["y"]
    "out_lift": {
        "left": dict(y=19),
    },
}


def get_axis_map(machine_type: str, orientation: str) -> dict:
    """
    获取指定设备类型和方向的完整轴映射字典

    Args:
        machine_type: 设备类型 ("in_up", "xn_side", "out_down", "out_up", "out_lift")
        orientation:  安装方向 ("left", "right")

    Returns:
        dict: 轴名称 → PLC Axis_List 索引
              例: {"z": 1, "y": 2, "x1": 3, "r1": 4}

    Raises:
        ValueError: 设备类型或方向未注册
    """
    if machine_type not in MACHINE_AXIS_MAP:
        raise ValueError(f"未知设备类型: {machine_type}, "
                         f"可用类型: {list(MACHINE_AXIS_MAP.keys())}")
    type_map = MACHINE_AXIS_MAP[machine_type]
    if orientation not in type_map:
        raise ValueError(f"设备 '{machine_type}' 不支持方向 '{orientation}', "
                         f"可用方向: {list(type_map.keys())}")
    return type_map[orientation]


def get_axis_index(machine_type: str, orientation: str, axis_name: str) -> int:
    """
    获取指定设备某个轴在 PLC Axis_List 中的索引

    Args:
        machine_type: 设备类型
        orientation:  安装方向
        axis_name:    轴名称 (如 "x", "z", "y", "x1"~"x6", "r1"~"r6")

    Returns:
        int: PLC Axis_List 索引

    Example:
        get_axis_index("xn_side", "right", "z")   → 1
        get_axis_index("xn_side", "right", "x3")  → 7
    """
    axis_map = get_axis_map(machine_type, orientation)
    if axis_name not in axis_map:
        raise ValueError(f"轴 '{axis_name}' 不存在于 {machine_type}/{orientation}, "
                         f"可用轴: {list(axis_map.keys())}")
    return axis_map[axis_name]


def get_x_axes(machine_type: str, orientation: str) -> dict:
    """
    获取所有 X 类轴的名称→索引映射

    对于内顶 (in_up):        返回 {"x": idx}
    对于侧面云雀 (xn_side):  返回 {"x1": idx, "x2": idx, ...}
    对于外底 (out_down):     返回 {"x": idx}

    Returns:
        dict: X轴名称 → PLC索引
    """
    axis_map = get_axis_map(machine_type, orientation)
    return {k: v for k, v in axis_map.items() if k.startswith("x")}


def has_axis(machine_type: str, orientation: str, axis_name: str) -> bool:
    """
    判断指定设备是否拥有某个轴

    Example:
        has_axis("xn_side", "right", "z")   → True
        has_axis("out_down", "left", "z")   → False
        has_axis("in_up", "right", "r1")    → False
    """
    axis_map = get_axis_map(machine_type, orientation)
    return axis_name in axis_map


def get_all_axis_indices(machine_type: str, orientation: str) -> list:
    """
    获取指定设备所有轴的 PLC 索引列表

    Returns:
        list[int]: 所有轴索引
    """
    return list(get_axis_map(machine_type, orientation).values())


def apply_to_axis_list(machine_data: dict, machine_type: str, orientation: str, axis_list: list):
    """
    将设备运动指令字典应用到 PLC AxisList 中

    Args:
        machine_data: 运动指令字典 {axis_name: AxisData}
        machine_type: 设备类型
        orientation:  安装方向
        axis_list:    PLC SendMovingFrameData.AxisList (长度100的列表)

    Example:
        # machine_data = {"z": AxisData(Pos=0, Speed=100), "y": AxisData(...), ...}
        apply_to_axis_list(machine_data, "xn_side", "right", send_frame.AxisList)
    """
    axis_map = get_axis_map(machine_type, orientation)
    for axis_name, axis_data in machine_data.items():
        if axis_name in axis_map:
            idx = axis_map[axis_name]
            axis_list[idx] = axis_data


def apply_device_axes_to_list(machine_config: dict, sn: int, axis_cmds: dict, axis_list: list):
    """
    按设备配置将单台设备的轴运动指令写入 AxisList。

    Args:
        machine_config: 全量 machine_config
        sn: 设备序列号
        axis_cmds: {axis_name: AxisData}
        axis_list: SendMovingFrameData.AxisList
    """
    machine_cfg = machine_config.get(str(sn))
    if not machine_cfg:
        return

    machine_type = machine_cfg.get("type", "")
    orientation = machine_cfg.get("install_orietation", "left")

    if machine_type == "out_lift":
        return

    try:
        apply_to_axis_list(axis_cmds, machine_type, orientation, axis_list)
    except Exception as e:
        logger.error(f"将 SN[{sn}] 的轴数据应用到 AxisList 时出错: {e}")
