import unittest
from types import SimpleNamespace

from model.formats.frame_by_frame.AxisFrameDataFormat import (
    AxisData as PointAxisData,
    AxisFrameData,
)
from model.motionplan.MachineAxisMap import get_axis_map
from model.motionplan.MotionManualOutFxPlanning import MotionManualOutFxPlanning
from model.motionplan.MotionOutFxFramePlanning import MotionOutFxFramePlanning
from model.motionplan.MotionReciprocate import MotionReciprocate
from model.motionplan.MotionToTarget import MotionToTarget
from model.motionplan.MotionXNSidePlanning import MotionXNSidePlanning
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
    if kind == "preposition":
        frames[0:5] = [populated_frame(rows) for _ in range(5)]
    elif kind == "start":
        frames[5:10] = [populated_frame(rows) for _ in range(5)]
    elif kind == "middle":
        frames[8:13] = [populated_frame(rows) for _ in range(5)]
    elif kind == "end":
        frames[11:16] = [populated_frame(rows) for _ in range(5)]
    return frames


class FakeMotionToTarget:
    def __init__(self):
        self.return_ready = False
        self.tolerance = 5

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

    def move_x_axes_to_target(self, machine_cfg, plc_data, target, speed, status=0):
        axis_map = get_axis_map(
            machine_cfg["type"],
            machine_cfg.get("install_orietation", "left"),
        )
        axis_cmds = {}
        all_ready = True
        for axis_name in machine_cfg["axis_type"]:
            if not axis_name.startswith("x"):
                continue
            current = int(plc_data.AxisList[axis_map[axis_name]].Pos)
            axis_cmds[axis_name] = AxisData(
                Pos=int(target),
                Speed=int(speed),
                Status=int(status),
            )
            if abs(current - int(target)) > self.tolerance:
                all_ready = False
        return axis_cmds, all_ready


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
                "x_pre_distance": 50,
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

    def test_idle_device_returns_all_axes_to_safe_when_reciprocation_disabled(self):
        self.set_axis_pos(self.machine_cfg, "z", 200)
        self.set_axis_pos(self.machine_cfg, "y", 50)
        self.set_axis_pos(self.machine_cfg, "x1", 300)

        axis_cmds, _, _ = self.planner.auto_out_fx_move(
            self.machine_cfg, self.runtime_cfg, self.plc, FrameQueueStub([])
        )

        self.assertEqual(axis_cmds["z"].Pos, 0)
        self.assertEqual(axis_cmds["y"].Pos, 0)
        self.assertEqual(axis_cmds["x1"].Pos, 0)

    def test_idle_device_reciprocates_y_while_x_and_z_return_safe(self):
        self.planner.spray_cfg["frame_idle_y_reciprocate_enabled"] = 1
        self.set_axis_pos(self.machine_cfg, "z", 200)
        self.set_axis_pos(self.machine_cfg, "y", 50)
        self.set_axis_pos(self.machine_cfg, "x1", 300)

        axis_cmds, _, _ = self.planner.auto_out_fx_move(
            self.machine_cfg, self.runtime_cfg, self.plc, FrameQueueStub([])
        )

        self.assertEqual(axis_cmds["z"].Pos, 0)
        self.assertEqual(axis_cmds["x1"].Pos, 0)
        self.assertEqual(axis_cmds["y"].Pos, self.runtime_cfg["y_move_min"])

        self.set_axis_pos(self.machine_cfg, "y", self.runtime_cfg["y_move_min"])
        axis_cmds, _, _ = self.planner.auto_out_fx_move(
            self.machine_cfg, self.runtime_cfg, self.plc, FrameQueueStub([])
        )
        self.assertEqual(axis_cmds["y"].Pos, self.runtime_cfg["y_move_max"])

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
        queue = FrameQueueStub(build_frames("start"))

        self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            queue,
        )
        self.assertEqual(self.planner._get_state(0).stage, "preposition")

        for axis_name in ("x1", "x2"):
            self.set_axis_pos(self.machine_cfg, axis_name, 300)
        self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            queue,
        )
        axis_cmds, _, _ = self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            queue,
        )
        self.assertEqual(self.planner._get_state(0).stage, "start")
        self.assertEqual(axis_cmds["x1"].Pos, axis_cmds["x2"].Pos)
        self.assertEqual(axis_cmds["z"], AxisData(Pos=900, Speed=100, Status=0))
        self.assertIn("y", axis_cmds)

    def test_tracking_prepositions_x_before_actual_start(self):
        runtime_cfg = dict(self.runtime_cfg, tracking=1, x_status_offset=50)
        preposition_queue = FrameQueueStub(build_frames("preposition"))

        axis_cmds, _, _ = self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            preposition_queue,
        )

        state = self.planner._get_state(0)
        self.assertEqual(state.stage, "preposition")
        self.assertEqual(axis_cmds["x1"].Pos, 300)
        self.assertEqual(axis_cmds["x2"].Pos, 300)
        self.assertEqual(axis_cmds["x1"].Status, 0)
        self.assertEqual(axis_cmds["y"], AxisData(Pos=100, Speed=100, Status=0))
        self.assertEqual(axis_cmds["z"], AxisData(Pos=0, Speed=0, Status=0))

        for axis_name in ("x1", "x2"):
            self.set_axis_pos(self.machine_cfg, axis_name, 300)
        self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            preposition_queue,
        )
        self.assertEqual(state.stage, "preposition")

        self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            FrameQueueStub(build_frames("start")),
        )
        self.assertEqual(state.stage, "start")

    def test_tracking_start_retract_holds_y_and_follows_z_until_x_arrives(self):
        runtime_cfg = dict(self.runtime_cfg, tracking=1, outside_total_cycles=1)
        state = self.planner._get_state(0)
        state.stage = "start"
        state.y_cycles = 1
        state.tracking_x_pre_target = 300
        self.set_axis_pos(self.machine_cfg, "y", 73)
        for axis_name in ("x1", "x2"):
            self.set_axis_pos(self.machine_cfg, axis_name, 700)

        self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            FrameQueueStub(build_frames("start")),
        )
        self.assertEqual(state.stage, "start_retract")

        retract_cmds, _, _ = self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            FrameQueueStub(build_frames("middle")),
        )
        self.assertEqual(state.stage, "start_retract")
        self.assertEqual(retract_cmds["x1"].Pos, 300)
        self.assertEqual(retract_cmds["x2"].Pos, 300)
        self.assertEqual(retract_cmds["y"], AxisData(Pos=73, Speed=0, Status=0))
        self.assertEqual(retract_cmds["z"], AxisData(Pos=900, Speed=100, Status=0))

        for axis_name in ("x1", "x2"):
            self.set_axis_pos(self.machine_cfg, axis_name, 300)
        self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            FrameQueueStub(build_frames("middle")),
        )
        self.assertEqual(state.stage, "middle")

        middle_cmds, _, _ = self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            FrameQueueStub(build_frames("middle")),
        )
        self.assertNotEqual(middle_cmds["y"].Speed, 0)
        self.assertEqual(middle_cmds["z"], AxisData(Pos=0, Speed=40, Status=0))

    def test_tracking_retract_keeps_x_gate_when_frames_are_temporarily_empty(self):
        runtime_cfg = dict(self.runtime_cfg, tracking=1)
        state = self.planner._get_state(0)
        state.stage = "start_retract"
        state.tracking_x_pre_target = 300
        self.set_axis_pos(self.machine_cfg, "y", 73)
        for axis_name in ("x1", "x2"):
            self.set_axis_pos(self.machine_cfg, axis_name, 700)

        axis_cmds, _, _ = self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            FrameQueueStub([]),
        )

        self.assertEqual(state.stage, "start_retract")
        self.assertEqual(axis_cmds["x1"].Pos, 300)
        self.assertEqual(axis_cmds["y"], AxisData(Pos=73, Speed=0, Status=0))
        self.assertEqual(axis_cmds["z"], AxisData(Pos=900, Speed=100, Status=0))
    def test_xn_side_retract_holds_all_six_y_axes_until_x_arrives(self):
        machine_cfg = dict(self.machine_cfg)
        machine_cfg.update(
            {
                "sn": 1,
                "type": "xn_side",
                "axis_type": [
                    "z",
                    "y1", "x1", "r1",
                    "y2", "x2", "r2",
                    "y3", "x3", "r3",
                    "y4", "x4", "r4",
                    "y5", "x5", "r5",
                    "y6", "x6", "r6",
                ],
                "origin_pos": [1000, 1500, 2000, 2500, 3000, 3500],
                "max_limit_speed": [250, 400, 500, 160],
                "min_limit_pos": [0, 0, 0, -180],
                "max_limit_pos": [900, 900, 1000, 180],
                "safe_pos": [0, 0, 0, 0],
            }
        )
        runtime_cfg = dict(self.runtime_cfg, tracking=1)
        state = self.planner._get_state(1)
        state.stage = "start_retract"
        state.tracking_x_pre_target = 300
        expected_y = {}
        for index in range(1, 7):
            y_position = index * 10
            expected_y[f"y{index}"] = y_position
            self.set_axis_pos(machine_cfg, f"y{index}", y_position)
            self.set_axis_pos(machine_cfg, f"x{index}", 700)

        axis_cmds, _, _ = self.planner.auto_out_fx_move(
            machine_cfg,
            runtime_cfg,
            self.plc,
            FrameQueueStub([]),
        )

        for axis_name, position in expected_y.items():
            with self.subTest(axis_name=axis_name):
                self.assertEqual(
                    axis_cmds[axis_name],
                    AxisData(Pos=position, Speed=0, Status=0),
                )
        for index in range(1, 7):
            self.assertEqual(axis_cmds[f"x{index}"].Pos, 300)
    def test_return_safe_finishes_when_frames_are_already_empty(self):
        state = self.planner._get_state(0)
        state.stage = "return_safe"
        self.motion_to_target.return_ready = True

        _, finished, _ = self.planner.auto_out_fx_move(
            self.machine_cfg,
            dict(self.runtime_cfg, tracking=1),
            self.plc,
            FrameQueueStub([]),
        )

        self.assertTrue(finished)
        self.assertEqual(self.planner._get_state(0).stage, "idle")
    def test_tracking_end_retracts_before_return_safe(self):
        runtime_cfg = dict(self.runtime_cfg, tracking=1, outside_total_cycles=1)
        state = self.planner._get_state(0)
        state.stage = "end"
        state.y_cycles = 1
        state.tracking_x_pre_target = 300
        self.set_axis_pos(self.machine_cfg, "y", 73)
        for axis_name in ("x1", "x2"):
            self.set_axis_pos(self.machine_cfg, axis_name, 700)

        self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            FrameQueueStub(build_frames("end")),
        )
        self.assertEqual(state.stage, "end_retract")

        retract_cmds, finished, _ = self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            FrameQueueStub(build_frames("end")),
        )
        self.assertFalse(finished)
        self.assertEqual(state.stage, "end_retract")
        self.assertEqual(retract_cmds["y"], AxisData(Pos=73, Speed=0, Status=0))
        self.assertEqual(retract_cmds["z"], AxisData(Pos=900, Speed=100, Status=0))

        for axis_name in ("x1", "x2"):
            self.set_axis_pos(self.machine_cfg, axis_name, 300)
        self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            FrameQueueStub(build_frames("end")),
        )
        self.assertEqual(state.stage, "return_safe")

    def test_motion_to_target_x_positioning_clamps_and_checks_all_x_axes(self):
        helper = MotionToTarget.__new__(MotionToTarget)
        helper.tolerance = 5
        self.set_axis_pos(self.machine_cfg, "x1", 100)
        self.set_axis_pos(self.machine_cfg, "x2", 0)

        axis_cmds, all_ready = helper.move_x_axes_to_target(
            self.machine_cfg,
            self.plc,
            target=-100,
            speed=200,
        )

        self.assertFalse(all_ready)
        self.assertEqual(axis_cmds["x1"], AxisData(Pos=0, Speed=200, Status=0))
        self.assertEqual(axis_cmds["x2"], AxisData(Pos=0, Speed=200, Status=0))

        self.set_axis_pos(self.machine_cfg, "x1", 0)
        _, all_ready = helper.move_x_axes_to_target(
            self.machine_cfg,
            self.plc,
            target=-100,
            speed=200,
        )
        self.assertTrue(all_ready)

    def test_tracking_z_follow_speed_respects_chain_status(self):
        state = self.planner._get_state(0)
        runtime_cfg = dict(self.runtime_cfg, tracking=1)
        self.plc.ChainSpeed = 100
        self.plc.ChainStatus = "stopped"

        for stage in ("start", "end", "start_retract", "end_retract"):
            with self.subTest(stage=stage):
                state.stage = stage
                command = self.planner._build_z_axis(
                    self.machine_cfg,
                    runtime_cfg,
                    self.plc,
                    state,
                    tracking=1,
                )
                self.assertEqual(command, AxisData(Pos=900, Speed=0, Status=0))

        state.stage = "start"
        self.plc.ChainStatus = "moving_reverse"
        command = self.planner._build_z_axis(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            state,
            tracking=1,
        )
        self.assertEqual(command, AxisData(Pos=900, Speed=0, Status=0))

        self.plc.ChainStatus = "moving_forward"
        command = self.planner._build_z_axis(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            state,
            tracking=1,
        )
        self.assertEqual(command, AxisData(Pos=900, Speed=100, Status=0))

    def test_chain_running_helpers_only_accept_forward_chain(self):
        helpers = (
            self.planner,
            MotionManualOutFxPlanning.__new__(MotionManualOutFxPlanning),
            MotionReciprocate.__new__(MotionReciprocate),
            MotionToTarget.__new__(MotionToTarget),
            MotionXNSidePlanning.__new__(MotionXNSidePlanning),
        )
        plc_data = SimpleNamespace(ChainStatus="moving_forward")

        for helper in helpers:
            with self.subTest(helper=type(helper).__name__, status="moving_forward"):
                self.assertTrue(helper._is_chain_running(plc_data))

            plc_data.ChainStatus = "moving_reverse"
            with self.subTest(helper=type(helper).__name__, status="moving_reverse"):
                self.assertFalse(helper._is_chain_running(plc_data))

            plc_data.ChainStatus = "stopped"
            with self.subTest(helper=type(helper).__name__, status="stopped"):
                self.assertFalse(helper._is_chain_running(plc_data))

            plc_data.ChainStatus = "moving_forward"
    def test_all_z_follow_speed_helpers_only_accept_forward_chain(self):
        helpers = (
            MotionReciprocate.__new__(MotionReciprocate),
            MotionToTarget.__new__(MotionToTarget),
            MotionXNSidePlanning.__new__(MotionXNSidePlanning),
        )
        plc_data = SimpleNamespace(ChainSpeed=100, ChainStatus="moving_forward")

        for helper in helpers:
            with self.subTest(helper=type(helper).__name__, status="moving_forward"):
                self.assertEqual(helper._resolve_follow_z_speed(plc_data), 100)

            plc_data.ChainStatus = "moving_reverse"
            with self.subTest(helper=type(helper).__name__, status="moving_reverse"):
                self.assertEqual(helper._resolve_follow_z_speed(plc_data), 0)

            plc_data.ChainStatus = "stopped"
            with self.subTest(helper=type(helper).__name__, status="stopped"):
                self.assertEqual(helper._resolve_follow_z_speed(plc_data), 0)

            plc_data.ChainStatus = "moving_forward"
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
        state = self.planner._get_state(0)
        state.stage = "start"
        state.tracking_x_pre_target = 300

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
        self.assertEqual(state.stage, "start_retract")
        self.assertEqual(state.start_cycles, 1)

        for axis_name in ("x1", "x2"):
            self.set_axis_pos(self.machine_cfg, axis_name, 300)
        self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            FrameQueueStub(build_frames("middle")),
        )
        self.assertEqual(state.stage, "middle")

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
        self.assertEqual(state.stage, "end_retract")
        self.assertEqual(state.end_cycles, 1)

        for axis_name in ("x1", "x2"):
            self.set_axis_pos(self.machine_cfg, axis_name, 300)
        self.planner.auto_out_fx_move(
            self.machine_cfg, runtime_cfg, self.plc, end_queue
        )
        self.assertEqual(state.stage, "return_safe")

    def test_tracking_can_use_x_as_cycle_axis(self):
        self.planner.spray_cfg["side_2d_cycle_axis"] = "x"
        runtime_cfg = dict(self.runtime_cfg, tracking=1, outside_total_cycles=1)
        queue = FrameQueueStub(build_frames("start"))
        state = self.planner._get_state(0)
        state.stage = "start"
        state.tracking_x_pre_target = 300

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

        self.assertEqual(state.start_cycles, 1)
        self.assertEqual(state.stage, "start_retract")

        for axis_name in ("x1", "x2"):
            self.set_axis_pos(self.machine_cfg, axis_name, 300)
        self.planner.auto_out_fx_move(
            self.machine_cfg,
            runtime_cfg,
            self.plc,
            FrameQueueStub(build_frames("middle")),
        )
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

    def test_nontracking_without_slow_in_out_uses_front_offset(self):
        self.planner.spray_cfg["frame_x_slow_in_out_enabled"] = 0

        for stage, frame_kind in (("start", "start"), ("middle", "middle"), ("end", "end")):
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

                self.assertEqual(offset, self.runtime_cfg["out_front_x_offset"])

    def test_basic_nontracking_uses_sparse_data_without_stage_detection(self):
        self.planner.spray_cfg["frame_x_slow_in_out_enabled"] = 0
        self.planner.spray_cfg["stage_detect_frame_count"] = 0
        frames = [empty_frame() for _ in range(30)]
        frames[10] = populated_frame()

        for interpolation_enabled, expected_positions in ((0, (350, 350)), (1, (350, 450))):
            with self.subTest(interpolation_enabled=interpolation_enabled):
                self.planner.reset_motion_state()
                self.planner.spray_cfg["frame_x_interpolation_enabled"] = interpolation_enabled

                axis_cmds, workpiece_complete, config_error = self.planner.auto_out_fx_move(
                    self.machine_cfg,
                    self.runtime_cfg,
                    self.plc,
                    FrameQueueStub(frames),
                )

                state = self.planner._get_state(0)
                self.assertEqual(state.stage, "idle")
                self.assertFalse(workpiece_complete)
                self.assertFalse(config_error)
                self.assertEqual(
                    (axis_cmds["x1"].Pos, axis_cmds["x2"].Pos),
                    expected_positions,
                )

    def test_basic_nontracking_zero_front_offset_uses_safe_default_100(self):
        self.planner.spray_cfg["frame_x_slow_in_out_enabled"] = 0
        self.planner.spray_cfg["frame_x_interpolation_enabled"] = 0
        machine_cfg = dict(self.machine_cfg, out_front_x_offset=0)
        runtime_cfg = dict(self.runtime_cfg, out_front_x_offset=0)
        frames = [empty_frame() for _ in range(30)]
        frames[10] = populated_frame()

        axis_cmds, _, config_error = self.planner.auto_out_fx_move(
            machine_cfg,
            runtime_cfg,
            self.plc,
            FrameQueueStub(frames),
        )

        self.assertFalse(config_error)
        self.assertEqual((axis_cmds["x1"].Pos, axis_cmds["x2"].Pos), (300, 300))

    def test_basic_nontracking_returns_x_safe_and_controls_idle_y_reciprocation(self):
        self.planner.spray_cfg["frame_x_slow_in_out_enabled"] = 0
        frames = [empty_frame() for _ in range(30)]
        for axis_name in ("x1", "x2"):
            self.set_axis_pos(self.machine_cfg, axis_name, 300)

        for reciprocate_enabled, expected_y in ((0, 0), (1, 100)):
            with self.subTest(reciprocate_enabled=reciprocate_enabled):
                self.planner.reset_motion_state()
                self.planner.spray_cfg["frame_idle_y_reciprocate_enabled"] = reciprocate_enabled

                axis_cmds, _, config_error = self.planner.auto_out_fx_move(
                    self.machine_cfg,
                    self.runtime_cfg,
                    self.plc,
                    FrameQueueStub(frames),
                )

                self.assertFalse(config_error)
                self.assertEqual(axis_cmds["x1"].Pos, 0)
                self.assertEqual(axis_cmds["x2"].Pos, 0)
                self.assertEqual(axis_cmds["y"].Pos, expected_y)

    def test_tracking_still_requires_stage_signature_when_slow_in_out_is_disabled(self):
        self.planner.spray_cfg["frame_x_slow_in_out_enabled"] = 0
        frames = [empty_frame() for _ in range(30)]
        frames[10] = populated_frame()

        axis_cmds, _, config_error = self.planner.auto_out_fx_move(
            self.machine_cfg,
            dict(self.runtime_cfg, tracking=1),
            self.plc,
            FrameQueueStub(frames),
        )

        self.assertFalse(config_error)
        self.assertEqual(self.planner._get_state(0).stage, "idle")
        self.assertEqual(axis_cmds["x1"].Pos, 0)
        self.assertEqual(axis_cmds["x2"].Pos, 0)

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
