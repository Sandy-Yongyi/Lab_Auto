import unittest
from types import SimpleNamespace

from model.formats.frame_by_frame.AxisFrameDataFormat import (
    AxisData as PointAxisData,
    AxisFrameData,
)
from model.motionplan.MachineAxisMap import get_axis_map
from model.motionplan.MotionOutFxFramePlanning import MotionOutFxFramePlanning
from model.plc.MovingFrameData import AxisData, create_axis_list


def empty_frame():
    return AxisFrameData(
        FrameData=[PointAxisData(H_Axis=0, V_Axis_Max=0, V_Axis_Min=0)]
    )


def populated_frame(rows=None):
    rows = rows or [(1050, 500, 800), (1550, 600, 900)]
    return AxisFrameData(
        FrameData=[
            PointAxisData(H_Axis=y, V_Axis_Min=x_min, V_Axis_Max=x_max)
            for y, x_min, x_max in rows
        ]
    )


def build_frames(kind, rows=None):
    frames = [empty_frame() for _ in range(30)]
    if kind == "start":
        frames[5:10] = [populated_frame(rows) for _ in range(5)]
    elif kind == "middle":
        frames[8:13] = [populated_frame(rows) for _ in range(5)]
    elif kind == "end":
        frames[11:16] = [populated_frame(rows) for _ in range(5)]
    return frames


class FakeMotionToTarget:
    def __init__(self):
        self.return_ready = False

    @staticmethod
    def _get_axis_current_pos(plc_data, index):
        return int(plc_data.AxisList[index].Pos)

    def hold_current_position(self, machine_cfg, plc_data):
        axis_map = get_axis_map(
            machine_cfg["type"],
            machine_cfg.get("install_orietation", "left"),
        )
        return {
            axis_name: AxisData(Pos=plc_data.AxisList[axis_map[axis_name]].Pos)
            for axis_name in machine_cfg["axis_type"]
        }

    def move_to_origin_safe(self, machine_cfg, runtime_cfg, plc_data):
        return (
            {
                axis_name: AxisData(Pos=0, Speed=100, Status=0)
                for axis_name in machine_cfg["axis_type"]
            },
            self.return_ready,
        )


class FrameQueueStub:
    def __init__(self, frames, direction="left"):
        self.frame_stack = {direction: frames}


class MotionOutFxFramePlanningTests(unittest.TestCase):
    def setUp(self):
        self.motion_to_target = FakeMotionToTarget()
        self.planner = MotionOutFxFramePlanning(
            spray_cfg={
                "stage_detect_frame_count": 5,
                "frame_x_interpolation_enabled": 1,
                "side_2d_cycle_axis": "y",
                "spray_pos_tolerance": 5,
            },
            read_data_cfg={"z_threshold": 10, "y_threshold": 10},
            motion_to_target=self.motion_to_target,
        )
        self.machine_cfg = {
            "sn": 0,
            "type": "out_fx",
            "install_orietation": "left",
            "axis_type": ["z", "y", "x1", "x2"],
            "spray_num": 2,
            "origin_pos": [1000, 1500],
            "tracking": 0,
            "y_move_min": 0,
            "y_move_max": 100,
            "x_position": 100,
            "z_position": 100,
            "out_front_x_offset": 50,
            "out_after_x_offset": 50,
            "out_up_y_offset": 100,
            "out_down_y_offset": 0,
            "out_z_front_offset": 50,
            "out_z_after_offset": 50,
            "x_status_offset": 0,
            "x_pos_speed": 200,
            "x_recip_speed": 120,
            "y_pos_speed": 100,
            "y_recip_speed": 100,
            "z_back_speed": 40,
            "z_zeroing_speed": 80,
            "outside_total_cycles": 1,
            "max_limit_speed": [250, 400, 500],
            "min_limit_pos": [0, 0, 0],
            "max_limit_pos": [900, 430, 1000],
            "safe_pos": [0, 0, 0],
        }
        self.runtime_cfg = {
            key: value
            for key, value in self.machine_cfg.items()
            if key not in {"sn", "type", "install_orietation", "axis_type"}
        }
        self.plc = SimpleNamespace(
            AxisList=create_axis_list(),
            ChainSpeed=100,
            ChainStatus="moving_forward",
        )

    def set_axis_pos(self, machine_cfg, axis_name, position):
        axis_map = get_axis_map(
            machine_cfg["type"],
            machine_cfg.get("install_orietation", "left"),
        )
        self.plc.AxisList[axis_map[axis_name]].Pos = position

    def test_state_does_not_reenter_start_after_start_completion(self):
        state = self.planner._get_state(0)
        state.stage = "middle"
        self.planner._transition_for_signatures(
            state,
            start=True,
            center=False,
            end=False,
            empty=False,
            tracking=1,
        )
        self.assertEqual(state.stage, "middle")

    def test_state_and_interpolation_memory_are_isolated_by_sn(self):
        self.planner._get_state(0).interpolation_targets["x1"] = 100
        self.planner._get_state(0).stage = "middle"
        self.assertEqual(self.planner._get_state(1).stage, "idle")
        self.assertNotIn("x1", self.planner._get_state(1).interpolation_targets)

    def test_nontracking_stage_sequence_is_monotonic(self):
        queue = FrameQueueStub(build_frames("start"))
        self.planner.auto_out_fx_move(
            self.machine_cfg, self.runtime_cfg, self.plc, queue
        )
        self.assertEqual(self.planner._get_state(0).stage, "start")

        queue.frame_stack["left"] = build_frames("middle")
        self.planner.auto_out_fx_move(
            self.machine_cfg, self.runtime_cfg, self.plc, queue
        )
        self.assertEqual(self.planner._get_state(0).stage, "middle")

        queue.frame_stack["left"] = build_frames("end")
        self.planner.auto_out_fx_move(
            self.machine_cfg, self.runtime_cfg, self.plc, queue
        )
        self.assertEqual(self.planner._get_state(0).stage, "end")

        queue.frame_stack["left"] = build_frames("empty")
        _, finished, _ = self.planner.auto_out_fx_move(
            self.machine_cfg, self.runtime_cfg, self.plc, queue
        )
        self.assertFalse(finished)
        self.assertEqual(self.planner._get_state(0).stage, "return_safe")

        self.motion_to_target.return_ready = True
        _, finished, _ = self.planner.auto_out_fx_move(
            self.machine_cfg, self.runtime_cfg, self.plc, queue
        )
        self.assertTrue(finished)
        self.assertEqual(self.planner._get_state(0).stage, "idle")

    def test_xn_side_y_reversal_reads_only_y1_and_counts_full_cycle(self):
        machine_cfg = dict(self.machine_cfg)
        machine_cfg.update(
            {
                "sn": 1,
                "type": "xn_side",
                "axis_type": ["z", "y1", "x1", "r1", "y2", "x2", "r2"],
                "origin_pos": [1000, 1500],
                "max_limit_speed": [250, 400, 500, 160],
                "min_limit_pos": [0, 0, 0, -180],
                "max_limit_pos": [900, 430, 1000, 180],
                "safe_pos": [0, 0, 0, 0],
            }
        )
        state = self.planner._get_state(1)
        state.stage = "start"

        self.set_axis_pos(machine_cfg, "y1", 0)
        self.set_axis_pos(machine_cfg, "y2", 300)
        first = self.planner._build_y_reciprocate_axis(
            machine_cfg, self.runtime_cfg, self.plc, state
        )
        self.assertEqual(first.Pos, 100)

        self.set_axis_pos(machine_cfg, "y1", 100)
        second = self.planner._build_y_reciprocate_axis(
            machine_cfg, self.runtime_cfg, self.plc, state
        )
        self.assertEqual(second.Pos, 0)

        self.set_axis_pos(machine_cfg, "y1", 0)
        third = self.planner._build_y_reciprocate_axis(
            machine_cfg, self.runtime_cfg, self.plc, state
        )
        self.assertEqual(third.Pos, 100)
        self.assertEqual(state.y_cycles, 1)

    def test_static_global_range_uses_each_gun_fixed_y_band(self):
        frames = build_frames("middle")
        window = self.planner.frame_helper.build_window(
            self.machine_cfg, self.runtime_cfg, z_cur=0, frame_count=len(frames)
        )
        result = self.planner._calculate_static_global_x(
            self.machine_cfg, self.runtime_cfg, frames, window
        )
        self.assertEqual((result.global_x_min, result.global_x_max), (500, 900))
        self.assertEqual(result.valid_axes, {"x1", "x2"})

    def test_static_mode_middle_uses_unified_global_min_target(self):
        self.planner.spray_cfg["frame_x_interpolation_enabled"] = 0
        self.planner._get_state(0).stage = "middle"
        axis_cmds, _, _ = self.planner.auto_out_fx_move(
            self.machine_cfg,
            self.runtime_cfg,
            self.plc,
            FrameQueueStub(build_frames("middle")),
        )
        self.assertEqual(axis_cmds["x1"].Pos, 350)
        self.assertEqual(axis_cmds["x2"].Pos, 350)
        self.assertEqual(axis_cmds["x1"].Speed, 200)

    def test_dynamic_mode_updates_each_gun_target_and_speed_by_y_bin(self):
        state = self.planner._get_state(0)
        state.stage = "middle"
        queue = FrameQueueStub(build_frames("middle"))

        first_cmds, _, _ = self.planner.auto_out_fx_move(
            self.machine_cfg, self.runtime_cfg, self.plc, queue
        )
        self.assertEqual(first_cmds["x1"].Pos, 350)
        self.assertEqual(first_cmds["x2"].Pos, 450)
        self.assertEqual(first_cmds["x1"].Speed, 200)

        self.set_axis_pos(self.machine_cfg, "y", 100)
        queue.frame_stack["left"] = build_frames(
            "middle",
            rows=[(1100, 800, 900), (1600, 900, 1000)],
        )
        second_cmds, _, _ = self.planner.auto_out_fx_move(
            self.machine_cfg, self.runtime_cfg, self.plc, queue
        )
        self.assertEqual(second_cmds["x1"].Pos, 650)
        self.assertEqual(second_cmds["x2"].Pos, 750)
        self.assertEqual(second_cmds["x1"].Speed, 300)

        queue.frame_stack["left"] = build_frames(
            "middle",
            rows=[(1100, 1000, 1100), (1600, 1100, 1200)],
        )
        same_bin_cmds, _, _ = self.planner.auto_out_fx_move(
            self.machine_cfg, self.runtime_cfg, self.plc, queue
        )
        self.assertEqual(same_bin_cmds["x1"].Pos, 650)
        self.assertEqual(same_bin_cmds["x1"].Speed, 300)

    def test_missing_gun_data_holds_that_x_axis_and_disables_status(self):
        self.planner.spray_cfg["frame_x_interpolation_enabled"] = 0
        self.planner._get_state(0).stage = "middle"
        self.set_axis_pos(self.machine_cfg, "x2", 222)
        frames = build_frames("middle", rows=[(1050, 500, 800)])
        axis_cmds, _, _ = self.planner.auto_out_fx_move(
            self.machine_cfg,
            self.runtime_cfg,
            self.plc,
            FrameQueueStub(frames),
        )
        self.assertEqual(axis_cmds["x1"].Pos, 350)
        self.assertEqual(axis_cmds["x2"], AxisData(Pos=222, Speed=0, Status=0))

    def test_tracking_start_uses_static_x_reciprocation_and_follow_z(self):
        runtime_cfg = dict(self.runtime_cfg, tracking=1)
        axis_cmds, _, _ = self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            FrameQueueStub(build_frames("start")),
        )
        self.assertEqual(self.planner._get_state(0).stage, "start")
        self.assertEqual(axis_cmds["x1"].Pos, axis_cmds["x2"].Pos)
        self.assertEqual(axis_cmds["z"], AxisData(Pos=900, Speed=100, Status=0))
        self.assertIn("y", axis_cmds)

    def test_tracking_middle_returns_z_safe_with_back_speed(self):
        self.planner._get_state(0).stage = "middle"
        runtime_cfg = dict(self.runtime_cfg, tracking=1)
        axis_cmds, _, _ = self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            FrameQueueStub(build_frames("middle")),
        )
        self.assertEqual(axis_cmds["z"], AxisData(Pos=0, Speed=40, Status=0))

    def test_tracking_start_and_end_use_separate_y_cycle_counts(self):
        runtime_cfg = dict(self.runtime_cfg, tracking=1, outside_total_cycles=1)
        start_queue = FrameQueueStub(build_frames("start"))

        self.planner.auto_out_fx_move(
            self.machine_cfg, runtime_cfg, self.plc, start_queue
        )
        self.set_axis_pos(self.machine_cfg, "y", 100)
        self.planner.auto_out_fx_move(
            self.machine_cfg, runtime_cfg, self.plc, start_queue
        )
        self.set_axis_pos(self.machine_cfg, "y", 0)
        self.planner.auto_out_fx_move(
            self.machine_cfg, runtime_cfg, self.plc, start_queue
        )
        state = self.planner._get_state(0)
        self.assertEqual(state.stage, "middle")
        self.assertEqual(state.start_cycles, 1)

        end_queue = FrameQueueStub(build_frames("end"))
        self.planner.auto_out_fx_move(
            self.machine_cfg, runtime_cfg, self.plc, end_queue
        )
        self.assertEqual(state.stage, "end")
        self.assertEqual(state.end_cycles, 0)

        self.set_axis_pos(self.machine_cfg, "y", 100)
        self.planner.auto_out_fx_move(
            self.machine_cfg, runtime_cfg, self.plc, end_queue
        )
        self.set_axis_pos(self.machine_cfg, "y", 0)
        self.planner.auto_out_fx_move(
            self.machine_cfg, runtime_cfg, self.plc, end_queue
        )
        self.assertEqual(state.stage, "return_safe")
        self.assertEqual(state.end_cycles, 1)

    def test_tracking_can_use_x_as_cycle_axis(self):
        self.planner.spray_cfg["side_2d_cycle_axis"] = "x"
        runtime_cfg = dict(self.runtime_cfg, tracking=1, outside_total_cycles=1)
        queue = FrameQueueStub(build_frames("start"))

        self.planner.auto_out_fx_move(
            self.machine_cfg, runtime_cfg, self.plc, queue
        )
        for axis_name in ("x1", "x2"):
            self.set_axis_pos(self.machine_cfg, axis_name, 350)
        self.planner.auto_out_fx_move(
            self.machine_cfg, runtime_cfg, self.plc, queue
        )
        for axis_name in ("x1", "x2"):
            self.set_axis_pos(self.machine_cfg, axis_name, 750)
        self.planner.auto_out_fx_move(
            self.machine_cfg, runtime_cfg, self.plc, queue
        )
        for axis_name in ("x1", "x2"):
            self.set_axis_pos(self.machine_cfg, axis_name, 350)
        self.planner.auto_out_fx_move(
            self.machine_cfg, runtime_cfg, self.plc, queue
        )

        state = self.planner._get_state(0)
        self.assertEqual(state.start_cycles, 1)
        self.assertEqual(state.stage, "middle")

    def test_nontracking_z_stays_at_zero_and_slow_offset_keeps_three_sections(self):
        for stage, frame_kind, expected_offset in (
            ("start", "start", 40),
            ("middle", "middle", 50),
            ("end", "end", 40),
        ):
            with self.subTest(stage=stage):
                frames = build_frames(frame_kind)
                window = self.planner.frame_helper.build_window(
                    self.machine_cfg,
                    self.runtime_cfg,
                    z_cur=0,
                    frame_count=len(frames),
                )
                state = self.planner._get_state(0)
                state.stage = stage
                offset = self.planner._resolve_current_x_offset(
                    state,
                    tracking=0,
                    frames=frames,
                    window=window,
                    machine_cfg=self.machine_cfg,
                    runtime_cfg=self.runtime_cfg,
                )
                self.assertEqual(offset, expected_offset)

                axis_cmds, _, _ = self.planner.auto_out_fx_move(
                    self.machine_cfg,
                    self.runtime_cfg,
                    self.plc,
                    FrameQueueStub(frames),
                )
                self.assertEqual(axis_cmds["z"].Pos, 0)

    def test_x_status_offset_changes_status_timing_without_changing_target(self):
        self.planner.spray_cfg["frame_x_interpolation_enabled"] = 0
        state = self.planner._get_state(0)
        state.stage = "end"
        frames = build_frames("end")

        without_offset, _, _ = self.planner.auto_out_fx_move(
            self.machine_cfg,
            dict(self.runtime_cfg, x_status_offset=0),
            self.plc,
            FrameQueueStub(frames),
        )
        state.stage = "end"
        with_offset, _, _ = self.planner.auto_out_fx_move(
            self.machine_cfg,
            dict(self.runtime_cfg, x_status_offset=50),
            self.plc,
            FrameQueueStub(frames),
        )
        self.assertEqual(without_offset["x1"].Pos, with_offset["x1"].Pos)
        self.assertEqual(without_offset["x1"].Status, 1)
        self.assertEqual(with_offset["x1"].Status, 0)

    def test_xn_side_r_axes_are_commanded_to_zero(self):
        machine_cfg = dict(self.machine_cfg)
        machine_cfg.update(
            {
                "sn": 1,
                "type": "xn_side",
                "axis_type": ["z", "y1", "x1", "r1", "y2", "x2", "r2"],
                "origin_pos": [1000, 1500],
                "max_limit_speed": [250, 400, 500, 160],
                "min_limit_pos": [0, 0, 0, -180],
                "max_limit_pos": [900, 430, 1000, 180],
                "safe_pos": [0, 0, 0, 0],
            }
        )
        state = self.planner._get_state(1)
        state.stage = "middle"
        axis_cmds, _, _ = self.planner.auto_out_fx_move(
            machine_cfg,
            self.runtime_cfg,
            self.plc,
            FrameQueueStub(build_frames("middle")),
        )
        self.assertEqual(axis_cmds["r1"].Pos, 0)
        self.assertEqual(axis_cmds["r1"].Status, 0)
        self.assertEqual(axis_cmds["r2"].Pos, 0)

    def test_invalid_tracking_configuration_holds_and_requests_chain_stop(self):
        axis_cmds, _, stop_chain = self.planner.auto_out_fx_move(
            self.machine_cfg,
            dict(self.runtime_cfg, tracking=2),
            self.plc,
            FrameQueueStub(build_frames("start")),
        )
        self.assertTrue(stop_chain)
        self.assertTrue(all(command.Status == 0 for command in axis_cmds.values()))


if __name__ == "__main__":
    unittest.main()
