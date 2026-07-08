import os
from dataclasses import dataclass
from model.motionplan.MachineAxisMap import get_axis_position_limits, get_axis_safe_pos, get_axis_speed_limit
from model.motionplan.motionutil.AxisLimits import build_axis, clamp_to_limit_r, clamp_to_limit_yx, clamp_to_limit_z
from model.utils.TomlLoader import TomlLoader


@dataclass
class RectReciprocateState:
    step: int = 0
    cycle_count: int = 0
    initialized: bool = False


@dataclass
class YReciprocateState:
    initialized: bool = False
    phase: str = "to_lower"


class MotionReciprocate:
    """侧喷往复辅助。"""

    def __init__(self):
        self.spray_cfg = TomlLoader.load(os.getcwd() + "\\model\\tomls\\sprayconfig.toml")
        self.tolerance = int(self.spray_cfg.get("spray_pos_tolerance", 10) or 10)
        self.min_recip_distance = int(self.spray_cfg.get("min_recip_distance", 60) or 60)
        self._rect_states: dict[str, RectReciprocateState] = {}
        self._y_states: dict[str, YReciprocateState] = {}

    def reset_states(self, state_prefix=None):
        if state_prefix is None:
            self._rect_states.clear()
            self._y_states.clear()
            return

        prefix = str(state_prefix)
        self._rect_states = {key: value for key, value in self._rect_states.items() if not key.startswith(prefix)}
        self._y_states = {key: value for key, value in self._y_states.items() if not key.startswith(prefix)}

    def build_side_reciprocate(self, machine_cfg, runtime_cfg, plc_data, gun_group,
                               x_min, x_max, r_angle, rect_threshold, state_key, x_recip_status=1,
                               keep_x_status_when_chain_stopped=False, total_cycles_key=None):
        recip_mode = self._normalize_side_reciprocate_mode(self.spray_cfg.get("side_reciprocate_mode", "rect"))
        if recip_mode == "2d":
            return self._build_side_2d_reciprocate(
                machine_cfg=machine_cfg,
                runtime_cfg=runtime_cfg,
                plc_data=plc_data,
                gun_group=gun_group,
                x_min=x_min,
                x_max=x_max,
                r_angle=r_angle,
                state_key=state_key,
                x_recip_status=x_recip_status,
                keep_x_status_when_chain_stopped=keep_x_status_when_chain_stopped,
                total_cycles_key=total_cycles_key,
            )

        return self._build_side_rect_reciprocate(
            machine_cfg=machine_cfg,
            runtime_cfg=runtime_cfg,
            plc_data=plc_data,
            gun_group=gun_group,
            x_min=x_min,
            x_max=x_max,
            r_angle=r_angle,
            rect_threshold=rect_threshold,
            state_key=state_key,
            x_recip_status=x_recip_status,
            keep_x_status_when_chain_stopped=keep_x_status_when_chain_stopped,
            total_cycles_key=total_cycles_key,
        )

    def build_side_end_face_reciprocate(self, machine_cfg, runtime_cfg, plc_data, gun_group,
                                        x_target, y_min, y_max, r_angle, state_key, z_target, z_speed,
                                        x_active_status=2):
        axis_cmds = {}
        enabled_guns = self._get_enabled_guns(gun_group)
        spray_num = int(machine_cfg.get("spray_num", 0) or 0)
        y_min_limit, y_max_limit = get_axis_position_limits(machine_cfg, "y")
        y_speed = int(runtime_cfg.get("y_recip_speed", machine_cfg.get("y_recip_speed", 100)) or 100)
        y_speed_limit = get_axis_speed_limit(machine_cfg, "y")
        x_speed = int(runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", 100)) or 100)

        y_lower = clamp_to_limit_yx(int(y_min or 0), y_min_limit, y_max_limit)
        y_upper = clamp_to_limit_yx(int(y_max or 0), y_min_limit, y_max_limit)
        if y_lower > y_upper:
            y_lower, y_upper = y_upper, y_lower

        axis_cmds["z"] = self._build_z_axis(machine_cfg, z_target, z_speed)

        if not enabled_guns:
            axis_cmds["y"] = build_axis(y_upper, y_speed, 0, y_speed_limit)
            _, xr_axes = self._build_group_xr_axes(machine_cfg, plc_data, spray_num, {}, 0, x_speed, 0, 0)
            axis_cmds.update(xr_axes)
            return axis_cmds

        state = self._y_states.get(state_key)
        if state is None:
            state = YReciprocateState()
            self._y_states[state_key] = state

        enabled_by_id = {
            int(getattr(gun, "gun_id")): gun
            for gun in enabled_guns
            if getattr(gun, "gun_id", None) is not None
        }
        x_status = 0 if not state.initialized else self._resolve_x_status(plc_data, x_active_status)
        x_arrived, xr_axes = self._build_group_xr_axes(
            machine_cfg=machine_cfg,
            plc_data=plc_data,
            spray_num=spray_num,
            enabled_by_id=enabled_by_id,
            x_target=x_target,
            x_speed=x_speed,
            x_status=x_status,
            r_angle=r_angle,
        )
        axis_cmds.update(xr_axes)

        y_cur = self._get_axis_pos(machine_cfg, plc_data, "y")
        if not state.initialized:
            axis_cmds["y"] = build_axis(y_upper, y_speed, 0, y_speed_limit)
            if x_arrived and abs(y_cur - y_upper) <= self.tolerance:
                state.initialized = True
                state.phase = "to_lower"
            return axis_cmds

        if y_cur > y_upper and state.phase == "to_upper":
            state.phase = "to_lower"
        elif y_cur < y_lower and state.phase == "to_lower":
            state.phase = "to_upper"

        y_target = y_lower if state.phase == "to_lower" else y_upper
        axis_cmds["y"] = build_axis(y_target, y_speed, 0, y_speed_limit)
        if abs(y_cur - y_target) <= self.tolerance:
            state.phase = "to_upper" if state.phase == "to_lower" else "to_lower"
        return axis_cmds

    def _build_side_rect_reciprocate(self, machine_cfg, runtime_cfg, plc_data, gun_group,
                                     x_min, x_max, r_angle, rect_threshold, state_key, x_recip_status,
                                     keep_x_status_when_chain_stopped, total_cycles_key=None):
        enabled_guns = self._get_enabled_guns(gun_group)
        spray_num = int(machine_cfg.get("spray_num", 0) or 0)
        y_range = self._get_group_y_range(machine_cfg, enabled_guns, runtime_cfg)
        if y_range is None:
            axis_cmds = {"z": self._build_work_z_axis(machine_cfg, runtime_cfg, plc_data)}
            axis_cmds["y"] = self._build_idle_y_axis(machine_cfg, plc_data)
            _, xr_axes = self._build_group_xr_axes(machine_cfg, plc_data, spray_num, {}, 0, 0, 0, 0)
            axis_cmds.update(xr_axes)
            return axis_cmds, True

        y_min, y_max = y_range
        x_min_target, x_max_target = self._clamp_x_range(machine_cfg, x_min, x_max)
        forward_seq, reverse_seq = self._build_rect_sequence(y_min, y_max, x_min_target, x_max_target, rect_threshold)

        state = self._rect_states.get(state_key)
        if state is None:
            state = RectReciprocateState()
            self._rect_states[state_key] = state

        total_cycles = self._get_total_cycles(runtime_cfg, machine_cfg, total_cycles_key)
        if state.cycle_count >= total_cycles:
            axis_cmds = {"z": self._build_work_z_axis(machine_cfg, runtime_cfg, plc_data)}
            axis_cmds["y"] = self._build_y_axis(machine_cfg, y_min, 0)
            enabled_by_id = {
                int(getattr(gun, "gun_id")): gun
                for gun in enabled_guns
                if getattr(gun, "gun_id", None) is not None
            }
            _, xr_axes = self._build_group_xr_axes(machine_cfg, plc_data, spray_num, enabled_by_id, x_min_target, self._get_x_speed(runtime_cfg, machine_cfg), 0, r_angle)
            axis_cmds.update(xr_axes)
            return axis_cmds, True

        sequence = forward_seq if state.cycle_count % 2 == 0 else reverse_seq
        if state.step >= len(sequence):
            state.step = 0
            state.cycle_count += 1
            return self._build_side_rect_reciprocate(
                machine_cfg=machine_cfg,
                runtime_cfg=runtime_cfg,
                plc_data=plc_data,
                gun_group=gun_group,
                x_min=x_min_target,
                x_max=x_max_target,
                r_angle=r_angle,
                rect_threshold=rect_threshold,
                state_key=state_key,
                x_recip_status=x_recip_status,
                keep_x_status_when_chain_stopped=keep_x_status_when_chain_stopped,
                total_cycles_key=total_cycles_key,
            )

        action, target = sequence[state.step]
        axis_cmds = {"z": self._build_work_z_axis(machine_cfg, runtime_cfg, plc_data)}
        y_cur = self._get_axis_pos(machine_cfg, plc_data, "y")
        enabled_by_id = {
            int(getattr(gun, "gun_id")): gun
            for gun in enabled_guns
            if getattr(gun, "gun_id", None) is not None
        }
        x_speed = self._get_x_speed(runtime_cfg, machine_cfg)
        y_speed = self._get_y_speed(runtime_cfg, machine_cfg)
        x_status = self._resolve_x_status(plc_data, x_recip_status, keep_x_status_when_chain_stopped)

        if action == "Y":
            axis_cmds["y"] = self._build_y_axis(machine_cfg, target, y_speed)
            _, xr_axes = self._build_group_xr_axes(machine_cfg, plc_data, spray_num, enabled_by_id, 0, x_speed, x_status, r_angle, hold_current=True)
            axis_cmds.update(xr_axes)
            if abs(y_cur - target) <= self.tolerance:
                state.step += 1
            return axis_cmds, False

        axis_cmds["y"] = self._build_y_axis(machine_cfg, y_cur, 0)
        x_arrived, xr_axes = self._build_group_xr_axes(machine_cfg, plc_data, spray_num, enabled_by_id, target, x_speed, x_status, r_angle)
        axis_cmds.update(xr_axes)
        if x_arrived:
            state.step += 1
        return axis_cmds, False

    def _build_side_2d_reciprocate(self, machine_cfg, runtime_cfg, plc_data, gun_group, x_min, x_max, r_angle, state_key,
                                   x_recip_status, keep_x_status_when_chain_stopped, total_cycles_key=None):
        enabled_guns = self._get_enabled_guns(gun_group)
        spray_num = int(machine_cfg.get("spray_num", 0) or 0)
        y_range = self._get_group_y_range(machine_cfg, enabled_guns, runtime_cfg)
        if y_range is None:
            axis_cmds = {"z": self._build_work_z_axis(machine_cfg, runtime_cfg, plc_data)}
            axis_cmds["y"] = self._build_idle_y_axis(machine_cfg, plc_data)
            _, xr_axes = self._build_group_xr_axes(machine_cfg, plc_data, spray_num, {}, 0, 0, 0, 0)
            axis_cmds.update(xr_axes)
            return axis_cmds, True

        y_min, y_max = y_range
        x_min_target, x_max_target = self._clamp_x_range(machine_cfg, x_min, x_max)
        total_cycles = self._get_total_cycles(runtime_cfg, machine_cfg, total_cycles_key)
        x_speed = self._get_x_speed(runtime_cfg, machine_cfg)
        x_position_speed = int(runtime_cfg.get("x_pos_speed", machine_cfg.get("x_pos_speed", x_speed)) or x_speed)
        y_speed = self._get_y_speed(runtime_cfg, machine_cfg)
        cycle_axis = self._normalize_side_2d_cycle_axis(self.spray_cfg.get("side_2d_cycle_axis", "x"))
        y_state_key = f"{state_key}:y"
        x_state_key = f"{state_key}:x"

        y_state = self._rect_states.get(y_state_key)
        if y_state is None:
            y_state = RectReciprocateState()
            self._rect_states[y_state_key] = y_state
        x_state = self._rect_states.get(x_state_key)
        if x_state is None:
            x_state = RectReciprocateState()
            self._rect_states[x_state_key] = x_state

        axis_cmds = {"z": self._build_work_z_axis(machine_cfg, runtime_cfg, plc_data)}
        enabled_by_id = {
            int(getattr(gun, "gun_id")): gun
            for gun in enabled_guns
            if getattr(gun, "gun_id", None) is not None
        }
        y_cur = self._get_axis_pos(machine_cfg, plc_data, "y")

        if not x_state.initialized:
            y_init_target = y_min if cycle_axis == "y" else y_cur
            y_init_speed = y_speed if cycle_axis == "y" else 0
            axis_cmds["y"] = self._build_y_axis(machine_cfg, y_init_target, y_init_speed)
            x_arrived, xr_axes = self._build_group_xr_axes(machine_cfg, plc_data, spray_num, enabled_by_id, x_min_target, x_position_speed, 0, r_angle)
            axis_cmds.update(xr_axes)
            y_arrived = abs(y_cur - y_init_target) <= self.tolerance
            if x_arrived and y_arrived:
                x_state.initialized = True
                y_state.initialized = True
                x_state.step = 0
                x_state.cycle_count = 0
                y_state.step = 0
                y_state.cycle_count = 0
            return axis_cmds, False

        cycle_count = y_state.cycle_count if cycle_axis == "y" else x_state.cycle_count
        if cycle_count >= total_cycles:
            axis_cmds["y"] = self._build_y_axis(machine_cfg, y_cur, 0)
            _, xr_axes = self._build_group_xr_axes(machine_cfg, plc_data, spray_num, enabled_by_id, x_min_target, x_speed, 0, r_angle)
            axis_cmds.update(xr_axes)
            return axis_cmds, True

        y_targets = [y_max, y_min]
        x_targets = [x_max_target, x_min_target]
        y_done = False

        y_target = y_targets[y_state.step % len(y_targets)]
        axis_cmds["y"] = self._build_y_axis(machine_cfg, y_target, y_speed)
        if abs(y_cur - y_target) <= self.tolerance:
            y_state.step = (y_state.step + 1) % len(y_targets)
            if y_state.step == 0:
                y_state.cycle_count += 1
                if cycle_axis == "y" and y_state.cycle_count >= total_cycles:
                    y_done = True

        x_target = x_targets[x_state.step % len(x_targets)]
        x_status = self._resolve_x_status(plc_data, x_recip_status, keep_x_status_when_chain_stopped)
        x_arrived, xr_axes = self._build_group_xr_axes(machine_cfg, plc_data, spray_num, enabled_by_id, x_target, x_speed, x_status, r_angle)
        axis_cmds.update(xr_axes)
        if x_arrived:
            x_state.step += 1
            if x_state.step >= len(x_targets):
                x_state.step = 0
                x_state.cycle_count += 1
                if cycle_axis == "x" and x_state.cycle_count >= total_cycles:
                    return axis_cmds, True

        return axis_cmds, y_done

    def _get_enabled_guns(self, gun_group):
        if gun_group is None:
            return []
        return [gun for gun in getattr(gun_group, "gundata_list", None) or [] if int(getattr(gun, "gun_y_enable", 0) or 0) == 1]

    def _get_group_y_range(self, machine_cfg, enabled_guns, runtime_cfg=None):
        if not enabled_guns:
            return None
        y_min_limit, y_max_limit = get_axis_position_limits(machine_cfg, "y")
        y_downer = [int(getattr(gun, "gun_y_downer", 0) or 0) for gun in enabled_guns]
        y_upper = [int(getattr(gun, "gun_y_upper", 0) or 0) for gun in enabled_guns]
        y_min = clamp_to_limit_yx(max(y_downer), y_min_limit, y_max_limit)
        raw_y_max = clamp_to_limit_yx(min(y_upper), y_min_limit, y_max_limit)
        reduce_distance = 0 if runtime_cfg is None else max(
            0,
            int(runtime_cfg.get("recip_reduce_distance", machine_cfg.get("recip_reduce_distance", 0)) or 0),
        )
        reduced_y_max = int(raw_y_max) - reduce_distance
        if reduced_y_max <= y_min:
            y_max = y_min
        else:
            y_max = clamp_to_limit_yx(reduced_y_max, y_min_limit, y_max_limit)
        if y_min > y_max:
            y_min, y_max = y_max, y_min
        return y_min, y_max

    def _build_group_xr_axes(self, machine_cfg, plc_data, spray_num, enabled_by_id,
                             x_target, x_speed, x_status, r_angle, hold_current=False):
        axis_cmds = {}
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")
        r_min_limit, r_max_limit = get_axis_position_limits(machine_cfg, "r")
        x_speed_limit = get_axis_speed_limit(machine_cfg, "x")
        r_speed_limit = get_axis_speed_limit(machine_cfg, "r")
        r_speed = r_speed_limit
        x_safe = clamp_to_limit_yx(get_axis_safe_pos(machine_cfg, "x", default=0), x_min_limit, x_max_limit)
        r_safe = clamp_to_limit_r(get_axis_safe_pos(machine_cfg, "r", default=0), r_min_limit, r_max_limit)
        x_target = clamp_to_limit_yx(int(x_target or 0), x_min_limit, x_max_limit)

        arrived = True
        for gun_idx in range(spray_num):
            axis_idx = gun_idx + 1
            x_name = f"x{axis_idx}"
            r_name = f"r{axis_idx}"
            gun = enabled_by_id.get(gun_idx)
            x_cur = self._get_axis_pos(machine_cfg, plc_data, x_name)
            gun_enabled = gun is not None and int(getattr(gun, "gun_y_enable", 0) or 0) == 1

            if not gun_enabled:
                axis_cmds[x_name] = build_axis(x_safe, x_speed, 0, x_speed_limit)
                axis_cmds[r_name] = build_axis(r_safe, r_speed, 0, r_speed_limit)
                continue

            final_x_target = x_cur if hold_current else x_target
            axis_cmds[x_name] = build_axis(final_x_target, x_speed, x_status, x_speed_limit)
            axis_cmds[r_name] = build_axis(clamp_to_limit_r(r_angle, r_min_limit, r_max_limit), r_speed, 0, r_speed_limit)
            if abs(x_cur - x_target) > self.tolerance:
                arrived = False

        return arrived, axis_cmds

    def _build_work_z_axis(self, machine_cfg, runtime_cfg, plc_data):
        _, z_max_limit = get_axis_position_limits(machine_cfg, "z")
        chain_speed = self._resolve_follow_z_speed(plc_data)
        return self._build_z_axis(machine_cfg, z_max_limit, chain_speed)

    def _build_z_axis(self, machine_cfg, target, speed):
        z_min_limit, z_max_limit = get_axis_position_limits(machine_cfg, "z")
        z_speed_limit = get_axis_speed_limit(machine_cfg, "z")
        z_target = clamp_to_limit_z(int(target or 0), z_min_limit, z_max_limit)
        return build_axis(z_target, speed, 0, z_speed_limit)

    def _build_idle_y_axis(self, machine_cfg, plc_data):
        y_cur = self._get_axis_pos(machine_cfg, plc_data, "y")
        return self._build_y_axis(machine_cfg, y_cur, 0)

    def _build_y_axis(self, machine_cfg, target, speed):
        y_min_limit, y_max_limit = get_axis_position_limits(machine_cfg, "y")
        y_speed_limit = get_axis_speed_limit(machine_cfg, "y")
        y_target = clamp_to_limit_yx(int(target or 0), y_min_limit, y_max_limit)
        return build_axis(y_target, speed, 0, y_speed_limit)

    def _get_axis_pos(self, machine_cfg, plc_data, axis_name):
        from model.motionplan.MachineAxisMap import get_axis_map

        axis_map = get_axis_map(machine_cfg.get("type", ""), machine_cfg.get("install_orietation", "right"))
        idx = axis_map[axis_name]
        axis_item = plc_data.AxisList[idx]
        if hasattr(axis_item, "Pos"):
            return int(getattr(axis_item, "Pos", 0) or 0)
        if isinstance(axis_item, (list, tuple)) and len(axis_item) > 0:
            return int(axis_item[0] or 0)
        if isinstance(axis_item, dict):
            return int(axis_item.get("Pos", 0) or 0)
        return 0

    def _get_x_speed(self, runtime_cfg, machine_cfg):
        return int(runtime_cfg.get("x_recip_speed", machine_cfg.get("x_recip_speed", 100)) or 100)

    def _get_y_speed(self, runtime_cfg, machine_cfg):
        return int(runtime_cfg.get("y_recip_speed", machine_cfg.get("y_recip_speed", 100)) or 100)

    def _resolve_x_status(self, plc_data, active_status, keep_when_chain_stopped=False):
        if self._is_chain_running(plc_data):
            return int(active_status or 0)
        return int(active_status or 0) if keep_when_chain_stopped else 0

    def _resolve_follow_z_speed(self, plc_data):
        chain_speed = int(getattr(plc_data, "ChainSpeed", 0) or 0)
        if not self._is_chain_running(plc_data):
            return 0
        return chain_speed if chain_speed != 0 else 0

    @staticmethod
    def _is_chain_running(plc_data):
        return getattr(plc_data, "ChainStatus", "stopped") in ("moving_forward", "moving_reverse")

    def _clamp_x_range(self, machine_cfg, x_min, x_max):
        x_min_limit, x_max_limit = get_axis_position_limits(machine_cfg, "x")
        x_min_target = clamp_to_limit_yx(int(x_min or 0), x_min_limit, x_max_limit)
        x_max_target = clamp_to_limit_yx(int(x_max or 0), x_min_limit, x_max_limit)
        if x_min_target > x_max_target:
            x_min_target, x_max_target = x_max_target, x_min_target
        return x_min_target, x_max_target

    def _build_rect_sequence(self, y_min, y_max, x_min, x_max, rect_threshold):
        total_range = max(0, y_max - y_min)
        base_seg = total_range / 4 if total_range else 0
        segment_cnt = 6 if base_seg > rect_threshold else 4
        step = total_range / segment_cnt if segment_cnt > 0 else 0
        y_targets = [int(y_min + step * i) for i in range(segment_cnt + 1)]
        y_targets[-1] = y_max

        forward = []
        x_flag = True
        for y_target in y_targets[1:]:
            forward.append(("X", x_max if x_flag else x_min))
            forward.append(("Y", y_target))
            x_flag = not x_flag

        reverse = []
        half_step = step / 2
        y_targets_reverse = [int(y_max - half_step - step * i) for i in range(segment_cnt)]
        y_targets_reverse.append(y_min)
        x_flag = True
        reverse.append(("X", x_max))
        for y_target in y_targets_reverse:
            reverse.append(("Y", y_target))
            reverse.append(("X", x_min if x_flag else x_max))
            x_flag = not x_flag
        return forward, reverse

    def _normalize_side_reciprocate_mode(self, recip_mode):
        if recip_mode is None:
            return "rect"
        recip_mode = str(recip_mode).strip().lower()
        if recip_mode in {"2d", "plane", "planar"}:
            return "2d"
        return "rect"

    def _normalize_side_2d_cycle_axis(self, cycle_axis):
        if cycle_axis is None:
            return "x"
        cycle_axis = str(cycle_axis).strip().lower()
        if cycle_axis == "y":
            return "y"
        return "x"

    @staticmethod
    def _get_total_cycles(runtime_cfg, machine_cfg, total_cycles_key=None):
        key = str(total_cycles_key or "")
        value = runtime_cfg.get(key, machine_cfg.get(key, None)) if key else None
        return int(value or 1)
