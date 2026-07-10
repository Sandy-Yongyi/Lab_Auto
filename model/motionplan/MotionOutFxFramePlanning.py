import os
from model.utils.LoggerUtil import logger
from model.utils.TomlLoader import TomlLoader
from model.motionplan.motionutil.AxisLimits import build_axis, clamp_to_limit_yx
from model.motionplan.MachineAxisMap import get_axis_position_limits, get_axis_speed_limit, get_axis_map
from model.motionplan.MotionToTarget import MotionToTarget
from model.motionplan.motionutil.FrameSearchHelper import FrameSearchHelper


class MotionOutFxPlanning:
    """外侧仿形自动喷涂规划。"""

    def __init__(self):
        self.spray_cfg = TomlLoader.load(os.path.join(os.getcwd(), "model", "tomls", "SprayConfig.toml"))
        self.read_data_cfg = TomlLoader.load(os.path.join(os.getcwd(), "model", "tomls", "ReadDataConfig.toml"))
        self.motion_to_target = MotionToTarget()
        self.z_threshold = int(self.read_data_cfg.get("z_threshold", 10))
        self.spray_pos_tolerance = int(self.spray_cfg.get("spray_pos_tolerance", 10))
        self.frame_helper = FrameSearchHelper(z_threshold=self.z_threshold)
        self._work_states = {}

    def reset_motion_state(self, sn=None):
        if sn is None:
            self._work_states = {}
            return
        self._work_states.pop(int(sn), None)

    def auto_out_fx_move(self, machine_cfg, runtime_cfg, plc_data, frame_queue_manager):
        sn = int(machine_cfg.get("sn", 0))
        state = self._work_states.setdefault(sn, self._create_initial_state())
        frames = self.frame_helper.get_side_frames(machine_cfg, frame_queue_manager)
        chain_running = self._is_chain_running(plc_data)

        if not frames:
            logger.warning(f"SN[{sn}] out_fx side frames not ready")
            return self.motion_to_target.hold_current_position(machine_cfg, plc_data), False, False

        axis_cmds = self.motion_to_target.hold_current_position(machine_cfg, plc_data)
        z_start_idx, z_end_idx = self._get_z_window(machine_cfg, runtime_cfg)
        y_min_abs, y_max_abs = self.frame_helper.scan_y_range(frames, z_start_idx, z_end_idx)

        if y_min_abs is None or y_max_abs is None:
            axis_cmds, _ = self.motion_to_target.move_to_origin_safe(machine_cfg, runtime_cfg, plc_data)
            logger.info(f"SN[{sn}] out_fx found no valid Y data in Z window, call move_to_origin_safe to return to safe position")
            return axis_cmds, False, False

        y_min_target, y_max_target = self._calc_y_motion_range(machine_cfg, runtime_cfg, y_min_abs, y_max_abs)
        axis_cmds["y"] = self._build_y_reciprocate_axis(
            machine_cfg,
            runtime_cfg,
            plc_data,
            state,
            y_min_target,
            y_max_target,
        )

        x_cmds = self._build_x_axis_commands(
            machine_cfg,
            runtime_cfg,
            chain_running,
            frames,
            z_start_idx,
            z_end_idx,
            y_min_target,
            y_max_target,
        )
        axis_cmds.update(x_cmds)

        logger.info(
            f"SN[{sn}] out_fx active, chain_running={chain_running}, "
            f"y_range=({y_min_target}, {y_max_target}), z_window=({z_start_idx}, {z_end_idx}), x_cmds={x_cmds}"
        )
        return axis_cmds, False, False

    def _create_initial_state(self):
        return {"y_phase": "to_max"}

    def _get_z_window(self, machine_cfg, runtime_cfg):
        z_position = int(machine_cfg.get("z_position", 0))
        z_front_offset = int(runtime_cfg.get("out_z_front_offset", machine_cfg.get("out_z_front_offset", 0)))
        z_after_offset = int(runtime_cfg.get("out_z_after_offset", machine_cfg.get("out_z_after_offset", 0)))
        start_idx = max(0, int((z_position - z_front_offset) / self.z_threshold))
        end_idx = max(0, int((z_position + z_after_offset) / self.z_threshold))
        return start_idx, end_idx

    def _calc_y_motion_range(self, machine_cfg, runtime_cfg, y_min_abs, y_max_abs):
        y_min_limit, y_max_limit = get_axis_position_limits(machine_cfg, "y")
        y_offset = int(runtime_cfg.get("out_up_y_offset", machine_cfg.get("out_up_y_offset", 0)))
        origin_pos = machine_cfg.get("origin_pos", [])
        origin_values = [int(v or 0) for v in origin_pos]
        y_origin_min = min(origin_values) if origin_values else 0
        y_origin_max = max(origin_values) if origin_values else 0
        gun_distance = abs(origin_values[1] - origin_values[0]) if len(origin_values) >= 2 else 0

        y_min_target = clamp_to_limit_yx(int(y_min_abs) - y_offset - y_origin_min, y_min_limit, y_max_limit)
        y_max_target = clamp_to_limit_yx(int(y_max_abs) + y_offset - y_origin_max, y_min_limit, y_max_limit)
        if y_max_target < gun_distance + y_min_target:
            y_max_target = clamp_to_limit_yx(gun_distance + y_min_target, y_min_limit, y_max_limit)
        return y_min_target, y_max_target

    def _build_y_reciprocate_axis(self, machine_cfg, runtime_cfg, plc_data, state, y_min_target, y_max_target):
        y_speed = int(runtime_cfg.get("y_recip_speed", machine_cfg.get("y_recip_speed", 100)))
        y_speed_limit = get_axis_speed_limit(machine_cfg, "y")
        y_min_limit, y_max_limit = get_axis_position_limits(machine_cfg, "y")
        y_target_min = clamp_to_limit_yx(y_min_target, y_min_limit, y_max_limit)
        y_target_max = clamp_to_limit_yx(y_max_target, y_min_limit, y_max_limit)
        if y_target_min > y_target_max:
            y_target_min, y_target_max = y_target_max, y_target_min

        y_cur = self._get_axis_pos(machine_cfg, plc_data, "y")
        phase = state.get("y_phase", "to_max")
        target = y_target_max if phase == "to_max" else y_target_min

        if y_cur > y_target_max:
            state["y_phase"] = "to_min"
            target = y_target_min
            return build_axis(target, y_speed, 0, y_speed_limit)

        if abs(y_cur - target) <= self.spray_pos_tolerance:
            phase = "to_min" if phase == "to_max" else "to_max"
            state["y_phase"] = phase
            target = y_target_max if phase == "to_max" else y_target_min

        return build_axis(target, y_speed, 0, y_speed_limit)

    def _build_x_axis_commands(self, machine_cfg, runtime_cfg, chain_running, frames, z_start_idx, z_end_idx, y_min_target, y_max_target):
        x_speed = int(runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 300)))
        x_position = int(machine_cfg.get("x_position", 850))
        x_offset = int(runtime_cfg.get("out_front_x_offset", machine_cfg.get("out_front_x_offset", 100)))
        fx_mode = int(self.spray_cfg.get("fx_mode", 0) or 0)
        x_speed_limit = get_axis_speed_limit(machine_cfg, "x")
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")
        origin_pos = machine_cfg.get("origin_pos", [])
        x_axis_names = [axis_name for axis_name in machine_cfg.get("axis_type", []) if axis_name.startswith("x")]

        results = []
        valid_targets = []
        for idx, axis_name in enumerate(x_axis_names):
            origin_y = int(origin_pos[idx] or 0) if idx < len(origin_pos) else 0
            y_start_abs = origin_y + y_min_target
            y_end_abs = origin_y + y_max_target
            x_min_values = self.frame_helper.collect_x_min_values(frames, z_start_idx, z_end_idx, y_start_abs, y_end_abs)
            # logger.debug(f"x_min_values={x_min_values}")
            has_x_min_data = len(x_min_values) > 0
            if has_x_min_data:
                x_target = clamp_to_limit_yx(min(x_min_values) - x_position - x_offset, x_min_limit, x_max_limit)
                # logger.debug(f"x_target={x_target}")
                valid_targets.append(x_target)
                results.append((axis_name, x_target, True))
            else:
                results.append((axis_name, 0, False))

        fallback_target = min(valid_targets) if valid_targets else 0

        # 二维模式：先保留原有逐轴搜索逻辑，再将 x1~x5 的目标统一收敛到所有有效目标中的最小值
        if fx_mode == 1 and x_axis_names:
            unified_target = fallback_target
            unified_status = 1 if chain_running and bool(valid_targets) else 0
            axis_cmds = {
                axis_name: build_axis(unified_target, x_speed, unified_status, x_speed_limit)
                for axis_name in x_axis_names
            }
            # logger.debug(
            #     f"out_fx fx_mode=1, unified x target applied: target={unified_target}, "
            #     f"status={unified_status}, valid_targets={valid_targets}"
            # )
            return axis_cmds

        axis_cmds = {}
        for axis_name, x_target, has_x_min_data in results:
            final_target = x_target if has_x_min_data else fallback_target
            status = 1 if chain_running and has_x_min_data else 0
            axis_cmds[axis_name] = build_axis(final_target, x_speed, status, x_speed_limit)
        return axis_cmds

    def _get_axis_pos(self, machine_cfg, plc_data, axis_name):
        axis_map = get_axis_map(machine_cfg.get("type", ""), machine_cfg.get("install_orietation", "left"))
        return self.motion_to_target._get_axis_current_pos(plc_data, axis_map[axis_name])

    def _is_chain_running(self, plc_data):
        return getattr(plc_data, "ChainStatus", "stopped") in ("moving_forward", "moving_reverse")
