import os
from dataclasses import dataclass, field

from model.utils.LoggerUtil import logger
from model.motionplan.motionutil.AxisLimits import build_axis, clamp_to_limit_yx
from model.motionplan.MachineAxisMap import (
    get_axis_map,
    get_axis_position_limits,
    get_axis_safe_pos,
    get_axis_speed_limit,
)
from model.motionplan.motionutil.FrameSearchHelper import FrameSearchHelper, FrameWindow
from model.motionplan.motionutil.FrameXMotionHelper import FrameXMotionHelper


@dataclass
class StaticGlobalXResult:
    global_x_min: int | None = None
    global_x_max: int | None = None
    gun_ranges: dict[str, tuple[int, int] | None] = field(default_factory=dict)
    search_y_ranges: dict[str, tuple[int, int]] = field(default_factory=dict)

    @property
    def valid_axes(self) -> set[str]:
        return {
            axis_name
            for axis_name, x_range in self.gun_ranges.items()
            if x_range is not None
        }


@dataclass
class DeviceFrameMotionState:
    stage: str = "idle"
    y_phase: str = "to_min"
    x_phase: str = "to_min"
    y_initialized: bool = False
    x_initialized: bool = False
    y_cycles: int = 0
    x_cycles: int = 0
    start_cycles: int = 0
    end_cycles: int = 0
    interpolation_bins: dict[str, int] = field(default_factory=dict)
    interpolation_y: dict[str, int] = field(default_factory=dict)
    interpolation_targets: dict[str, int] = field(default_factory=dict)
    interpolation_speeds: dict[str, int] = field(default_factory=dict)
    interpolation_base_x: dict[str, int] = field(default_factory=dict)


class MotionOutFxFramePlanning:
    """外侧仿形自动喷涂规划。"""

    def __init__(self, spray_cfg=None, read_data_cfg=None, motion_to_target=None):
        if spray_cfg is None or read_data_cfg is None:
            from model.utils.TomlLoader import TomlLoader

            config_dir = os.path.join(os.getcwd(), "model", "tomls")
            if spray_cfg is None:
                spray_cfg = TomlLoader.load(os.path.join(config_dir, "SprayConfig.toml"))
            if read_data_cfg is None:
                read_data_cfg = TomlLoader.load(os.path.join(config_dir, "ReadDataConfig.toml"))
        if motion_to_target is None:
            from model.motionplan.MotionToTarget import MotionToTarget

            motion_to_target = MotionToTarget()

        self.spray_cfg = spray_cfg
        self.read_data_cfg = read_data_cfg
        self.motion_to_target = motion_to_target
        self.z_threshold = int(self.read_data_cfg.get("z_threshold", 10))
        self.y_threshold = int(self.read_data_cfg.get("y_threshold", 10) or 10)
        if self.y_threshold <= 0:
            raise ValueError(f"y_threshold 必须大于 0，当前值: {self.y_threshold}")
        self.spray_pos_tolerance = int(self.spray_cfg.get("spray_pos_tolerance", 10))
        self.frame_helper = FrameSearchHelper(z_threshold=self.z_threshold)
        self.x_motion_helper = FrameXMotionHelper()
        self._work_states = {}

    def reset_motion_state(self, sn=None):
        if sn is None:
            self._work_states = {}
            return
        self._work_states.pop(int(sn), None)

    def auto_out_fx_move(self, machine_cfg, runtime_cfg, plc_data, frame_queue_manager):
        sn = int(machine_cfg.get("sn", 0))
        state = self._get_state(sn)
        frames = self.frame_helper.get_side_frames(machine_cfg, frame_queue_manager)
        chain_running = self._is_chain_running(plc_data)

        if not frames:
            logger.warning(f"SN[{sn}] frame side data not ready")
            return self.motion_to_target.hold_current_position(machine_cfg, plc_data), False, False

        z_cur = self._get_axis_pos(machine_cfg, plc_data, "z")
        window = self.frame_helper.build_window(
            machine_cfg,
            runtime_cfg,
            z_cur=z_cur,
            frame_count=len(frames),
        )
        tracking = int(runtime_cfg.get("tracking", machine_cfg.get("tracking", 0)) or 0)
        config_error = self._validate_motion_config(
            machine_cfg,
            runtime_cfg,
            window,
            tracking,
        )
        if config_error:
            logger.error(f"SN[{sn}] frame motion config error: {config_error}")
            axis_cmds = self.motion_to_target.hold_current_position(machine_cfg, plc_data)
            return axis_cmds, False, True

        detect_count = int(self.spray_cfg.get("stage_detect_frame_count", 5) or 0)
        start_signature = self.frame_helper.has_start_signature(
            frames, window, detect_count
        )
        end_signature = self.frame_helper.has_end_signature(
            frames, window, detect_count
        )
        center_has_data = self.frame_helper.frame_has_data(
            self.frame_helper.get_frame_by_index(frames, window.center)
        )
        window_empty = self.frame_helper.window_is_empty(frames, window)
        self._transition_for_signatures(
            state,
            start=start_signature,
            center=center_has_data,
            end=end_signature,
            empty=window_empty,
            tracking=tracking,
        )

        if state.stage == "idle":
            return self.motion_to_target.hold_current_position(machine_cfg, plc_data), False, False

        if state.stage == "return_safe":
            axis_cmds, all_ready = self.motion_to_target.move_to_origin_safe(
                machine_cfg,
                runtime_cfg,
                plc_data,
            )
            if all_ready:
                self._work_states[sn] = DeviceFrameMotionState()
            return axis_cmds, bool(all_ready), False

        axis_cmds = self.motion_to_target.hold_current_position(machine_cfg, plc_data)
        axis_cmds["y"] = self._build_y_reciprocate_axis(
            machine_cfg, runtime_cfg, plc_data, state
        )

        interpolation_enabled = int(
            self.spray_cfg.get("frame_x_interpolation_enabled", 1) or 0
        )
        if tracking and state.stage in {"start", "end"}:
            static_result = self._calculate_static_global_x(
                machine_cfg, runtime_cfg, frames, window
            )
            x_cmds = self._build_tracking_reciprocate_x_commands(
                machine_cfg,
                runtime_cfg,
                plc_data,
                state,
                frames,
                window,
                static_result,
                chain_running,
            )
        else:
            current_x_offset = self._resolve_current_x_offset(
                state,
                tracking,
                frames,
                window,
                machine_cfg,
                runtime_cfg,
            )
            if interpolation_enabled == 1:
                x_cmds = self._build_dynamic_x_commands(
                    machine_cfg,
                    runtime_cfg,
                    plc_data,
                    state,
                    frames,
                    window,
                    current_x_offset,
                    chain_running,
                )
            else:
                static_result = self._calculate_static_global_x(
                    machine_cfg, runtime_cfg, frames, window
                )
                x_cmds = self._build_static_position_x_commands(
                    machine_cfg,
                    runtime_cfg,
                    plc_data,
                    frames,
                    window,
                    static_result,
                    current_x_offset,
                    chain_running,
                )
        axis_cmds.update(x_cmds)
        axis_cmds["z"] = self._build_z_axis(
            machine_cfg, runtime_cfg, plc_data, state, tracking
        )
        axis_cmds.update(self._build_r_axis_commands(machine_cfg))

        self._complete_tracking_stage_if_needed(
            state,
            tracking,
            machine_cfg,
            runtime_cfg,
        )

        logger.info(
            f"SN[{sn}] frame stage={state.stage}, tracking={tracking}, "
            f"chain_running={chain_running}, z_window=({window.start}, {window.center}, {window.end})"
        )
        return axis_cmds, False, False

    def _get_state(self, sn: int) -> DeviceFrameMotionState:
        return self._work_states.setdefault(int(sn), DeviceFrameMotionState())

    def _validate_motion_config(self, machine_cfg, runtime_cfg, window, tracking):
        if tracking not in (0, 1):
            return f"tracking must be 0 or 1, current value: {tracking}"

        interpolation_enabled = int(
            self.spray_cfg.get("frame_x_interpolation_enabled", 1) or 0
        )
        if interpolation_enabled not in (0, 1):
            return (
                "frame_x_interpolation_enabled must be 0 or 1, "
                f"current value: {interpolation_enabled}"
            )

        cycle_axis = str(self.spray_cfg.get("side_2d_cycle_axis", "y")).lower()
        if cycle_axis not in {"x", "y"}:
            return f"side_2d_cycle_axis must be x or y, current value: {cycle_axis}"

        detect_count = int(self.spray_cfg.get("stage_detect_frame_count", 5) or 0)
        window_length = window.end - window.start + 1
        if detect_count <= 0 or detect_count >= window_length:
            return (
                "stage_detect_frame_count must be greater than 0 and smaller "
                f"than the Z window length, current values: {detect_count}/{window_length}"
            )

        total_cycles = int(
            runtime_cfg.get(
                "outside_total_cycles",
                machine_cfg.get("outside_total_cycles", 1),
            )
            or 0
        )
        if total_cycles <= 0:
            return f"outside_total_cycles must be greater than 0, current value: {total_cycles}"

        y_target_min, y_target_max = self._get_y_targets(machine_cfg, runtime_cfg)
        if y_target_min >= y_target_max:
            return (
                "y_move_min must be smaller than y_move_max after limit checking, "
                f"current values: {y_target_min}/{y_target_max}"
            )

        x_axis_names = self._get_x_axis_names(machine_cfg)
        origin_pos = machine_cfg.get("origin_pos", [])
        if len(origin_pos) < len(x_axis_names):
            return (
                "origin_pos count is smaller than X axis count, "
                f"current values: {len(origin_pos)}/{len(x_axis_names)}"
            )
        return None

    def _transition_for_signatures(
        self, state, *, start, center, end, empty, tracking
    ):
        if state.stage == "idle" and start:
            self._set_stage(state, "start", tracking)
        elif state.stage == "start" and not tracking and center:
            self._set_stage(state, "middle", tracking)
        elif state.stage == "middle" and end:
            self._set_stage(state, "end", tracking)
        elif state.stage == "end" and not tracking and empty:
            self._set_stage(state, "return_safe", tracking)

    def _set_stage(self, state, stage, tracking):
        if state.stage == stage:
            return

        state.stage = stage
        self._clear_interpolation_state(state)
        if stage == "start":
            self._reset_reciprocation_state(state)
            state.start_cycles = 0
            state.end_cycles = 0
        elif stage == "middle":
            state.x_phase = "to_min"
            state.x_initialized = False
            state.x_cycles = 0
        elif stage == "end":
            state.x_phase = "to_min"
            state.x_initialized = False
            state.x_cycles = 0
            state.end_cycles = 0
            if tracking:
                state.y_phase = "to_min"
                state.y_initialized = False
                state.y_cycles = 0
        elif stage in {"return_safe", "idle"}:
            self._reset_reciprocation_state(state)

    @staticmethod
    def _reset_reciprocation_state(state):
        state.y_phase = "to_min"
        state.x_phase = "to_min"
        state.y_initialized = False
        state.x_initialized = False
        state.y_cycles = 0
        state.x_cycles = 0

    @staticmethod
    def _clear_interpolation_state(state):
        state.interpolation_bins.clear()
        state.interpolation_y.clear()
        state.interpolation_targets.clear()
        state.interpolation_speeds.clear()
        state.interpolation_base_x.clear()

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

    def _build_y_reciprocate_axis(self, machine_cfg, runtime_cfg, plc_data, state):
        y_speed = int(runtime_cfg.get("y_recip_speed", machine_cfg.get("y_recip_speed", 100)))
        y_speed_limit = get_axis_speed_limit(machine_cfg, "y")
        y_target_min, y_target_max = self._get_y_targets(machine_cfg, runtime_cfg)
        y_axis_name = self._get_logical_y_axis_name(machine_cfg)
        y_cur = self._get_axis_pos(machine_cfg, plc_data, y_axis_name)

        if not state.y_initialized:
            if abs(y_cur - y_target_min) <= self.spray_pos_tolerance:
                state.y_initialized = True
                state.y_phase = "to_max"
                target = y_target_max
            else:
                state.y_phase = "to_min"
                target = y_target_min
        else:
            target = y_target_max if state.y_phase == "to_max" else y_target_min
            if abs(y_cur - target) <= self.spray_pos_tolerance:
                if state.y_phase == "to_max":
                    state.y_phase = "to_min"
                    target = y_target_min
                else:
                    state.y_cycles += 1
                    state.y_phase = "to_max"
                    target = y_target_max

        return build_axis(target, y_speed, 0, y_speed_limit)

    def _get_y_targets(self, machine_cfg, runtime_cfg):
        y_min_limit, y_max_limit = get_axis_position_limits(machine_cfg, "y")
        y_target_min = clamp_to_limit_yx(
            int(runtime_cfg.get("y_move_min", machine_cfg.get("y_move_min", 0)) or 0),
            y_min_limit,
            y_max_limit,
        )
        y_target_max = clamp_to_limit_yx(
            int(runtime_cfg.get("y_move_max", machine_cfg.get("y_move_max", 0)) or 0),
            y_min_limit,
            y_max_limit,
        )
        return y_target_min, y_target_max

    @staticmethod
    def _get_logical_y_axis_name(machine_cfg):
        return "y1" if machine_cfg.get("type") == "xn_side" else "y"

    @staticmethod
    def _get_x_axis_names(machine_cfg):
        return [
            axis_name
            for axis_name in machine_cfg.get("axis_type", [])
            if axis_name.startswith("x")
        ]

    def _calculate_static_global_x(self, machine_cfg, runtime_cfg, frames, window):
        y_move_min, y_move_max = self._get_y_targets(machine_cfg, runtime_cfg)
        out_down_y_offset = int(
            runtime_cfg.get(
                "out_down_y_offset",
                machine_cfg.get("out_down_y_offset", 0),
            )
            or 0
        )
        out_up_y_offset = int(
            runtime_cfg.get(
                "out_up_y_offset",
                machine_cfg.get("out_up_y_offset", 0),
            )
            or 0
        )
        origin_pos = machine_cfg.get("origin_pos", [])
        result = StaticGlobalXResult()

        for index, axis_name in enumerate(self._get_x_axis_names(machine_cfg)):
            search_y_min, search_y_max = self.x_motion_helper.build_static_search_y_range(
                origin_pos[index],
                y_move_min,
                y_move_max,
                out_down_y_offset,
                out_up_y_offset,
            )
            result.search_y_ranges[axis_name] = (search_y_min, search_y_max)
            x_min, x_max = self.frame_helper.collect_x_range(
                frames,
                window,
                search_y_min,
                search_y_max,
            )
            result.gun_ranges[axis_name] = (
                (int(x_min), int(x_max))
                if x_min is not None and x_max is not None
                else None
            )

        result.global_x_min, result.global_x_max = (
            self.x_motion_helper.aggregate_static_x_range(
                result.gun_ranges.values()
            )
        )
        return result

    def _build_tracking_reciprocate_x_commands(
        self,
        machine_cfg,
        runtime_cfg,
        plc_data,
        state,
        frames,
        window,
        static_result,
        chain_running,
    ):
        axis_names = self._get_x_axis_names(machine_cfg)
        if static_result.global_x_min is None or static_result.global_x_max is None:
            return {
                axis_name: self._build_hold_x_command(machine_cfg, plc_data, axis_name)
                for axis_name in axis_names
            }

        x_position = int(machine_cfg.get("x_position", 0) or 0)
        front_offset = int(
            runtime_cfg.get(
                "out_front_x_offset",
                machine_cfg.get("out_front_x_offset", 0),
            )
            or 0
        )
        after_offset = int(
            runtime_cfg.get(
                "out_after_x_offset",
                machine_cfg.get("out_after_x_offset", 0),
            )
            or 0
        )
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")
        target_min = self.x_motion_helper.build_final_x_target(
            static_result.global_x_min,
            x_position,
            front_offset,
            x_min_limit,
            x_max_limit,
        )
        target_max = self.x_motion_helper.build_final_x_target(
            static_result.global_x_max,
            x_position,
            after_offset,
            x_min_limit,
            x_max_limit,
        )
        if target_min > target_max:
            target_min, target_max = target_max, target_min

        valid_axes = static_result.valid_axes
        valid_positions = [
            self._get_axis_pos(machine_cfg, plc_data, axis_name)
            for axis_name in axis_names
            if axis_name in valid_axes
        ]
        x_pos_speed = int(
            runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 0)) or 0
        )
        x_recip_speed = int(
            runtime_cfg.get("x_recip_speed", machine_cfg.get("x_recip_speed", 0)) or 0
        )

        if not state.x_initialized:
            if valid_positions and all(
                abs(position - target_min) <= self.spray_pos_tolerance
                for position in valid_positions
            ):
                state.x_initialized = True
                state.x_phase = "to_max"
                target = target_max
                speed = x_recip_speed
            else:
                state.x_phase = "to_min"
                target = target_min
                speed = x_pos_speed
        else:
            target = target_max if state.x_phase == "to_max" else target_min
            if valid_positions and all(
                abs(position - target) <= self.spray_pos_tolerance
                for position in valid_positions
            ):
                if state.x_phase == "to_max":
                    state.x_phase = "to_min"
                    target = target_min
                else:
                    state.x_cycles += 1
                    state.x_phase = "to_max"
                    target = target_max
            speed = x_recip_speed

        x_speed_limit = get_axis_speed_limit(machine_cfg, "x")
        axis_cmds = {}
        for axis_name in axis_names:
            if axis_name not in valid_axes:
                axis_cmds[axis_name] = self._build_hold_x_command(
                    machine_cfg, plc_data, axis_name
                )
                continue
            search_y_min, search_y_max = static_result.search_y_ranges[axis_name]
            status = int(
                chain_running
                and self._has_x_status_data(
                    frames,
                    window,
                    search_y_min,
                    search_y_max,
                    runtime_cfg,
                    machine_cfg,
                )
            )
            axis_cmds[axis_name] = build_axis(
                target,
                speed,
                status,
                x_speed_limit,
            )
        return axis_cmds

    def _build_static_position_x_commands(
        self,
        machine_cfg,
        runtime_cfg,
        plc_data,
        frames,
        window,
        static_result,
        current_x_offset,
        chain_running,
    ):
        axis_names = self._get_x_axis_names(machine_cfg)
        if static_result.global_x_min is None:
            return {
                axis_name: self._build_hold_x_command(machine_cfg, plc_data, axis_name)
                for axis_name in axis_names
            }

        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")
        target = self.x_motion_helper.build_final_x_target(
            static_result.global_x_min,
            machine_cfg.get("x_position", 0),
            current_x_offset,
            x_min_limit,
            x_max_limit,
        )
        speed = int(
            runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 0)) or 0
        )
        speed_limit = get_axis_speed_limit(machine_cfg, "x")
        axis_cmds = {}
        for axis_name in axis_names:
            if axis_name not in static_result.valid_axes:
                axis_cmds[axis_name] = self._build_hold_x_command(
                    machine_cfg, plc_data, axis_name
                )
                continue
            search_y_min, search_y_max = static_result.search_y_ranges[axis_name]
            status = int(
                chain_running
                and self._has_x_status_data(
                    frames,
                    window,
                    search_y_min,
                    search_y_max,
                    runtime_cfg,
                    machine_cfg,
                )
            )
            axis_cmds[axis_name] = build_axis(target, speed, status, speed_limit)
        return axis_cmds

    def _build_dynamic_x_commands(
        self,
        machine_cfg,
        runtime_cfg,
        plc_data,
        state,
        frames,
        window,
        current_x_offset,
        chain_running,
    ):
        y_axis_name = self._get_logical_y_axis_name(machine_cfg)
        y_cur = self._get_axis_pos(machine_cfg, plc_data, y_axis_name)
        y_bin = int(y_cur / self.y_threshold)
        out_down_y_offset = int(
            runtime_cfg.get(
                "out_down_y_offset",
                machine_cfg.get("out_down_y_offset", 0),
            )
            or 0
        )
        out_up_y_offset = int(
            runtime_cfg.get(
                "out_up_y_offset",
                machine_cfg.get("out_up_y_offset", 0),
            )
            or 0
        )
        origin_pos = machine_cfg.get("origin_pos", [])
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")
        x_speed_limit = get_axis_speed_limit(machine_cfg, "x")
        x_pos_speed = int(
            runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 0)) or 0
        )
        y_recip_speed = int(
            runtime_cfg.get("y_recip_speed", machine_cfg.get("y_recip_speed", 0)) or 0
        )
        axis_cmds = {}

        for index, axis_name in enumerate(self._get_x_axis_names(machine_cfg)):
            search_y_min, search_y_max = self.x_motion_helper.build_dynamic_search_y_range(
                origin_pos[index],
                y_cur,
                out_down_y_offset,
                out_up_y_offset,
            )
            x_min_values = self.frame_helper.collect_x_min_values(
                frames,
                window.start,
                window.end,
                search_y_min,
                search_y_max,
            )
            if not x_min_values:
                axis_cmds[axis_name] = self._build_hold_x_command(
                    machine_cfg, plc_data, axis_name
                )
                continue

            if (
                axis_name not in state.interpolation_bins
                or state.interpolation_bins[axis_name] != y_bin
            ):
                base_x_min = min(x_min_values)
                target = self.x_motion_helper.build_final_x_target(
                    base_x_min,
                    machine_cfg.get("x_position", 0),
                    current_x_offset,
                    x_min_limit,
                    x_max_limit,
                )
                speed = self.x_motion_helper.calculate_interpolation_speed(
                    state.interpolation_y.get(axis_name),
                    y_cur,
                    state.interpolation_targets.get(axis_name),
                    target,
                    y_recip_speed,
                    x_speed_limit,
                    x_pos_speed,
                )
                state.interpolation_bins[axis_name] = y_bin
                state.interpolation_y[axis_name] = y_cur
                state.interpolation_base_x[axis_name] = base_x_min
                state.interpolation_speeds[axis_name] = speed
                state.interpolation_targets[axis_name] = target
            else:
                base_x_min = state.interpolation_base_x[axis_name]
                target = self.x_motion_helper.build_final_x_target(
                    base_x_min,
                    machine_cfg.get("x_position", 0),
                    current_x_offset,
                    x_min_limit,
                    x_max_limit,
                )
                speed = state.interpolation_speeds[axis_name]
                state.interpolation_targets[axis_name] = target

            status = int(
                chain_running
                and self._has_x_status_data(
                    frames,
                    window,
                    search_y_min,
                    search_y_max,
                    runtime_cfg,
                    machine_cfg,
                )
            )
            axis_cmds[axis_name] = build_axis(
                target,
                speed,
                status,
                x_speed_limit,
            )
        return axis_cmds

    def _resolve_current_x_offset(
        self, state, tracking, frames, window, machine_cfg, runtime_cfg
    ):
        front_offset = int(
            runtime_cfg.get(
                "out_front_x_offset",
                machine_cfg.get("out_front_x_offset", 0),
            )
            or 0
        )
        if tracking:
            return front_offset

        populated_indices = [
            index
            for index in range(window.start, window.end + 1)
            if self.frame_helper.frame_has_data(
                self.frame_helper.get_frame_by_index(frames, index)
            )
        ]
        if not populated_indices:
            return 0

        after_offset = int(
            runtime_cfg.get(
                "out_after_x_offset",
                machine_cfg.get("out_after_x_offset", 0),
            )
            or 0
        )
        return self.x_motion_helper.resolve_slow_offset(
            max(populated_indices) * self.z_threshold,
            min(populated_indices) * self.z_threshold,
            window.center * self.z_threshold,
            int(
                runtime_cfg.get(
                    "out_z_front_offset",
                    machine_cfg.get("out_z_front_offset", 0),
                )
                or 0
            ),
            int(
                runtime_cfg.get(
                    "out_z_after_offset",
                    machine_cfg.get("out_z_after_offset", 0),
                )
                or 0
            ),
            max(front_offset, after_offset),
        )

    def _build_z_axis(self, machine_cfg, runtime_cfg, plc_data, state, tracking):
        z_min_limit, z_max_limit = get_axis_position_limits(machine_cfg, "z")
        z_speed_limit = get_axis_speed_limit(machine_cfg, "z")
        if tracking and state.stage in {"start", "end"}:
            target = z_max_limit
            speed = int(getattr(plc_data, "ChainSpeed", 0) or 0)
        elif tracking:
            target = clamp_to_limit_yx(
                get_axis_safe_pos(machine_cfg, "z"),
                z_min_limit,
                z_max_limit,
            )
            speed = int(
                runtime_cfg.get(
                    "z_back_speed",
                    machine_cfg.get("z_back_speed", 0),
                )
                or 0
            )
        else:
            target = clamp_to_limit_yx(0, z_min_limit, z_max_limit)
            speed = int(
                runtime_cfg.get(
                    "z_zeroing_speed",
                    machine_cfg.get("z_zeroing_speed", 0),
                )
                or 0
            )
        return build_axis(target, speed, 0, z_speed_limit)

    def _build_r_axis_commands(self, machine_cfg):
        axis_cmds = {}
        for axis_name in machine_cfg.get("axis_type", []):
            if not axis_name.startswith("r"):
                continue
            r_min_limit, r_max_limit = get_axis_position_limits(machine_cfg, axis_name)
            axis_cmds[axis_name] = build_axis(
                clamp_to_limit_yx(0, r_min_limit, r_max_limit),
                0,
                0,
                get_axis_speed_limit(machine_cfg, axis_name),
            )
        return axis_cmds

    def _complete_tracking_stage_if_needed(
        self, state, tracking, machine_cfg, runtime_cfg
    ):
        if not tracking or state.stage not in {"start", "end"}:
            return

        cycle_axis = str(self.spray_cfg.get("side_2d_cycle_axis", "y")).lower()
        completed_cycles = state.x_cycles if cycle_axis == "x" else state.y_cycles
        total_cycles = int(
            runtime_cfg.get(
                "outside_total_cycles",
                machine_cfg.get("outside_total_cycles", 1),
            )
            or 0
        )
        if state.stage == "start":
            state.start_cycles = completed_cycles
            if state.start_cycles >= total_cycles:
                self._set_stage(state, "middle", tracking)
        else:
            state.end_cycles = completed_cycles
            if state.end_cycles >= total_cycles:
                self._set_stage(state, "return_safe", tracking)

    def _build_hold_x_command(self, machine_cfg, plc_data, axis_name):
        current_position = self._get_axis_pos(machine_cfg, plc_data, axis_name)
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, axis_name)
        return build_axis(
            clamp_to_limit_yx(current_position, x_min_limit, x_max_limit),
            0,
            0,
            get_axis_speed_limit(machine_cfg, axis_name),
        )

    def _has_x_status_data(
        self,
        frames,
        window,
        search_y_min,
        search_y_max,
        runtime_cfg,
        machine_cfg,
    ):
        x_status_offset = int(
            runtime_cfg.get(
                "x_status_offset",
                machine_cfg.get("x_status_offset", 0),
            )
            or 0
        )
        offset_frames = max(0, int(x_status_offset / self.z_threshold))
        start_index = max(0, window.start - offset_frames)
        end_index = max(start_index, window.end - offset_frames)
        return bool(
            self.frame_helper.collect_x_min_values(
                frames,
                start_index,
                end_index,
                min(search_y_min, search_y_max),
                max(search_y_min, search_y_max),
            )
        )

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
