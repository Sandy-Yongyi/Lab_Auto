import unittest
import tomllib

from model.motionplan.MachineAxisMap import (
    apply_device_axes_to_list,
    get_axis_index,
    get_axis_position_limits,
    get_axis_speed_limit,
)
from model.plc.MovingFrameData import AxisData, create_axis_list


class MachineAxisLimitTests(unittest.TestCase):
    def setUp(self):
        self.machine_config = {
            "0": {
                "type": "out_fx",
                "install_orietation": "left",
                "max_limit_speed": [250, 500, 500],
                "min_limit_pos": [0, -100, 10],
                "max_limit_pos": [900, 430, 1000],
            }
        }

    def test_clamps_position_and_speed_for_every_axis_type(self):
        axis_list = create_axis_list()
        apply_device_axes_to_list(
            self.machine_config,
            0,
            {
                "z": AxisData(Pos=999, Speed=999, Status=1),
                "y": AxisData(Pos=-500, Speed=600, Status=2),
                "x1": AxisData(Pos=2000, Speed=-20, Status=3),
            },
            axis_list,
        )

        self.assertEqual(axis_list[0], AxisData(Pos=900, Speed=250, Status=1))
        self.assertEqual(axis_list[1], AxisData(Pos=-100, Speed=500, Status=2))
        self.assertEqual(axis_list[2], AxisData(Pos=1000, Speed=0, Status=3))

    def test_logical_y_is_limited_before_broadcast(self):
        machine_config = {
            "1": {
                "type": "xn_side",
                "install_orietation": "left",
                "max_limit_speed": [250, 400, 500, 160],
                "min_limit_pos": [0, 0, 0, -180],
                "max_limit_pos": [900, 430, 1000, 180],
            }
        }
        axis_list = create_axis_list()
        apply_device_axes_to_list(
            machine_config,
            1,
            {"y": AxisData(Pos=500, Speed=600, Status=1)},
            axis_list,
        )

        for index in (11, 14, 17, 20, 23, 26):
            with self.subTest(index=index):
                self.assertEqual(axis_list[index], AxisData(Pos=430, Speed=400, Status=1))

    def test_all_48_configured_axes_are_limited_before_plc_output(self):
        with open("model/tomls/MachineConfig1.toml", "rb") as config_file:
            machine_config = tomllib.load(config_file)
        axis_list = create_axis_list()

        for sn in range(3):
            machine_cfg = machine_config[str(sn)]
            axis_cmds = {}
            for axis_name in machine_cfg["axis_type"]:
                _, max_position = get_axis_position_limits(machine_cfg, axis_name)
                max_speed = get_axis_speed_limit(machine_cfg, axis_name)
                axis_cmds[axis_name] = AxisData(
                    Pos=max_position + 100,
                    Speed=max_speed + 100,
                    Status=1,
                )
            apply_device_axes_to_list(machine_config, sn, axis_cmds, axis_list)

        checked_indices = set()
        for sn in range(3):
            machine_cfg = machine_config[str(sn)]
            for axis_name in machine_cfg["axis_type"]:
                index = get_axis_index(
                    machine_cfg["type"],
                    machine_cfg["install_orietation"],
                    axis_name,
                )
                _, max_position = get_axis_position_limits(machine_cfg, axis_name)
                max_speed = get_axis_speed_limit(machine_cfg, axis_name)
                self.assertEqual(
                    axis_list[index],
                    AxisData(Pos=max_position, Speed=max_speed, Status=1),
                )
                checked_indices.add(index)

        self.assertEqual(checked_indices, set(range(48)))


if __name__ == "__main__":
    unittest.main()
