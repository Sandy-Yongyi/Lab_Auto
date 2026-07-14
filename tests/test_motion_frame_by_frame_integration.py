import ast
import unittest
from pathlib import Path
from types import SimpleNamespace
import tomllib

from model.motionplan.MotionFrameByFramePlanning import MotionFrameByFramePlanning
from model.plc.MovingFrameData import AxisData, create_axis_list


class RecordingFramePlanner:
    def __init__(self, stop_sns=None):
        self.calls = []
        self.stop_sns = set(stop_sns or [])

    def auto_out_fx_move(
        self, machine_cfg, runtime_cfg, plc_data, frame_queue_manager
    ):
        sn = int(machine_cfg["sn"])
        self.calls.append(sn)
        value = (sn + 1) * 10
        axis_cmds = {
            "z": AxisData(Pos=value, Speed=value, Status=0),
            "y": AxisData(Pos=value, Speed=value, Status=0),
        }
        for axis_name in machine_cfg["axis_type"]:
            if axis_name.startswith(("x", "r")):
                axis_cmds[axis_name] = AxisData(
                    Pos=value,
                    Speed=value,
                    Status=0,
                )
        return axis_cmds, False, sn in self.stop_sns


class MotionTargetStub:
    def __init__(self):
        self.hold_calls = []
        self.safe_calls = []

    def hold_current_position(self, machine_cfg, plc_data):
        self.hold_calls.append(int(machine_cfg["sn"]))
        return {}

    def move_to_origin_safe(self, machine_cfg, runtime_cfg, plc_data):
        self.safe_calls.append(int(machine_cfg["sn"]))
        return {}, True


class ManualPlannerStub:
    def __init__(self):
        self.calls = []

    def auto_manual_out_fx_move(self, **kwargs):
        self.calls.append(int(kwargs["machine_cfg"]["sn"]))
        return {}


class MotionFrameByFrameIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open("model/tomls/MachineConfig1.toml", "rb") as config_file:
            cls.machine_config = tomllib.load(config_file)

    def setUp(self):
        self.frame_planner = RecordingFramePlanner()
        self.manual_planner = ManualPlannerStub()
        self.motion_target = MotionTargetStub()
        self.planner = MotionFrameByFramePlanning.__new__(MotionFrameByFramePlanning)
        self.planner.out_fx_planner = self.frame_planner
        self.planner.manual_out_fx_planner = self.manual_planner
        self.planner.motion_to_target = self.motion_target
        self.proc = SimpleNamespace(
            plc_data=SimpleNamespace(
                Operate=0x0F,
                Status=1,
                AxisList=create_axis_list(),
            ),
            num_devices=3,
            machine_config=self.machine_config,
            runtime_machine_config={0: {}, 1: {}, 2: {}},
            runtime_spray_config={},
            mode_config={"spray_mode": 0},
            frame_queue_manager=object(),
            last_operate_state=0x0F,
            device_returning_to_origin=[False, False, False],
            device_origin_complete={0: False, 1: False, 2: False},
            lidar_status=0,
            raw_data_timeout_active=False,
        )

    def test_automatic_path_has_no_machine_type_gate(self):
        source_path = Path("model/motionplan/MotionFrameByFramePlanning.py")
        source = source_path.read_text(encoding="utf-8")
        tree = ast.parse(source)
        build_method = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef) and node.name == "build_moving_frame"
        )
        build_source = ast.get_source_segment(source, build_method)
        self.assertNotIn('machine_type == "out_fx"', build_source)

    def test_all_three_devices_call_the_shared_frame_planner(self):
        self.planner.build_moving_frame(self.proc)
        self.assertEqual(self.frame_planner.calls, [0, 1, 2])

    def test_each_device_writes_only_its_own_axis_segment(self):
        moving_frame = self.planner.build_moving_frame(self.proc)
        positions = [axis.Pos for axis in moving_frame.AxisList]
        self.assertEqual(positions[0:10], [10] * 10)
        self.assertEqual(positions[10:29], [20] * 19)
        self.assertEqual(positions[29:48], [30] * 19)

    def test_logical_y_is_broadcast_to_all_six_xn_side_y_axes(self):
        moving_frame = self.planner.build_moving_frame(self.proc)
        for index in (11, 14, 17, 20, 23, 26):
            with self.subTest(index=index):
                self.assertEqual(moving_frame.AxisList[index].Pos, 20)
        for index in (30, 33, 36, 39, 42, 45):
            with self.subTest(index=index):
                self.assertEqual(moving_frame.AxisList[index].Pos, 30)

    def test_any_device_stop_request_stops_the_chain(self):
        self.frame_planner.stop_sns = {0}
        moving_frame = self.planner.build_moving_frame(self.proc)
        self.assertEqual(moving_frame.Operate, 0)

    def test_fault_branch_does_not_call_automatic_planner(self):
        self.proc.lidar_status = 1
        moving_frame = self.planner.build_moving_frame(self.proc)
        self.assertEqual(self.frame_planner.calls, [])
        self.assertEqual(moving_frame.Operate, 0)

    def test_single_disabled_device_returns_safe_without_blocking_others(self):
        self.proc.plc_data.Operate = 0x0B
        self.planner.build_moving_frame(self.proc)
        self.assertEqual(self.frame_planner.calls, [0, 2])
        self.assertEqual(self.motion_target.safe_calls, [1])

    def test_servo_alarm_returns_all_devices_safe(self):
        self.proc.plc_data.Status = 0
        self.planner.build_moving_frame(self.proc)
        self.assertEqual(self.frame_planner.calls, [])
        self.assertEqual(self.motion_target.safe_calls, [0, 1, 2])

    def test_raw_data_timeout_returns_safe_and_stops_chain(self):
        self.proc.raw_data_timeout_active = True
        moving_frame = self.planner.build_moving_frame(self.proc)
        self.assertEqual(self.frame_planner.calls, [])
        self.assertEqual(self.motion_target.safe_calls, [0, 1, 2])
        self.assertEqual(moving_frame.Operate, 0)

    def test_manual_mode_keeps_existing_out_fx_only_behavior(self):
        self.proc.mode_config["spray_mode"] = 1
        self.planner.build_moving_frame(self.proc)
        self.assertEqual(self.frame_planner.calls, [])
        self.assertEqual(self.manual_planner.calls, [0])
        self.assertEqual(self.motion_target.safe_calls, [1, 2])


if __name__ == "__main__":
    unittest.main()
