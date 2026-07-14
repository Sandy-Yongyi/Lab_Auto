import os

from model.utils.StrategyUtil import validate_strategy_name


MACHINE_CONFIG_FILE_BY_STRATEGY = {
    "frame_by_frame": "MachineConfig1.toml",
    "complete_workpiece": "MachineConfig2.toml",
    "continuous_bidirectional": "MachineConfig3.toml",
}


def get_machine_config_filename(strategy_name: str) -> str:
    """根据运动策略返回对应的设备配置文件名。"""
    strategy_name = validate_strategy_name(strategy_name)
    return MACHINE_CONFIG_FILE_BY_STRATEGY[strategy_name]


def get_machine_config_path(config_dir: str, strategy_name: str) -> str:
    """根据运动策略返回设备配置文件完整路径。"""
    return os.path.join(config_dir, get_machine_config_filename(strategy_name))


def validate_machine_params(values: dict, param_range_rules: dict):
    """校验界面参数范围及 Y 轴最小、最大位置关系。"""
    for key, value in values.items():
        if isinstance(value, dict):
            validate_machine_params(value, param_range_rules)
            continue
        if key not in param_range_rules:
            continue
        min_value, max_value = param_range_rules[key]
        if not (min_value <= value <= max_value):
            raise ValueError(f"{key} 超出范围 {min_value} ~ {max_value}")

    if "y_move_min" in values and "y_move_max" in values:
        if int(values["y_move_min"]) >= int(values["y_move_max"]):
            raise ValueError("y_move_min 必须小于 y_move_max")
