import ast
import tomllib
import unittest
from pathlib import Path

from model.utils.MachineConfigUtil import (
    get_machine_config_filename,
    validate_machine_params,
)


FRAME_BY_FRAME_PARAM_KEYS = [
    "tracking",
    "y_move_min",
    "y_move_max",
    "out_front_x_offset",
    "out_after_x_offset",
    "x_pos_speed",
    "x_recip_speed",
    "out_up_y_offset",
    "out_down_y_offset",
    "y_pos_speed",
    "y_recip_speed",
    "out_z_front_offset",
    "out_z_after_offset",
    "z_back_speed",
    "z_zeroing_speed",
    "x_status_offset",
    "outside_total_cycles",
]


def _class_constant(path: str, class_name: str, constant_name: str):
    tree = ast.parse(Path(path).read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            for statement in node.body:
                if not isinstance(statement, ast.Assign):
                    continue
                if any(
                    isinstance(target, ast.Name) and target.id == constant_name
                    for target in statement.targets
                ):
                    return ast.literal_eval(statement.value)
    raise AssertionError(f"未找到 {class_name}.{constant_name}")


class MachineConfigSelectionTests(unittest.TestCase):
    def test_maps_each_strategy_to_its_machine_config(self):
        self.assertEqual(get_machine_config_filename("frame_by_frame"), "MachineConfig1.toml")
        self.assertEqual(get_machine_config_filename("complete_workpiece"), "MachineConfig2.toml")
        self.assertEqual(get_machine_config_filename("continuous_bidirectional"), "MachineConfig3.toml")

    def test_rejects_unknown_strategy(self):
        with self.assertRaises(ValueError):
            get_machine_config_filename("unknown")

    def test_three_machine_config_files_exist_and_are_parseable(self):
        config_dir = Path("model/tomls")
        for index in (1, 2, 3):
            with self.subTest(index=index):
                with (config_dir / f"MachineConfig{index}.toml").open("rb") as config_file:
                    config = tomllib.load(config_file)
                self.assertEqual(set(config), {"0", "1", "2"})

    def test_frame_config_has_tracking_and_valid_y_defaults(self):
        with Path("model/tomls/MachineConfig1.toml").open("rb") as config_file:
            config = tomllib.load(config_file)
        for sn in ("0", "1", "2"):
            with self.subTest(sn=sn):
                self.assertEqual(config[sn]["tracking"], 0)
                self.assertEqual(config[sn]["y_move_min"], 0)
                self.assertEqual(config[sn]["y_move_max"], 430)

    def test_frame_ui_field_order_is_exact(self):
        actual = _class_constant(
            "view/MachineConfigFrame.py",
            "MachineConfigFrame",
            "FRAME_BY_FRAME_PARAM_KEYS",
        )
        self.assertEqual(actual, FRAME_BY_FRAME_PARAM_KEYS)

    def test_validates_tracking_and_y_range(self):
        rules = {
            "tracking": (0, 1),
            "y_move_min": (0, 430),
            "y_move_max": (0, 430),
        }
        validate_machine_params(
            {"tracking": 0, "y_move_min": 0, "y_move_max": 430},
            rules,
        )
        for values in (
            {"tracking": 2, "y_move_min": 0, "y_move_max": 430},
            {"tracking": 0, "y_move_min": -1, "y_move_max": 430},
            {"tracking": 0, "y_move_min": 300, "y_move_max": 200},
            {"tracking": 0, "y_move_min": 430, "y_move_max": 430},
        ):
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    validate_machine_params(values, rules)

    def test_runtime_and_ui_use_strategy_specific_paths(self):
        plc_source = Path("control/PlcCommunicationProcess.py").read_text(encoding="utf-8")
        ui_source = Path("control/MachineConfigFrameControl.py").read_text(encoding="utf-8")
        main_source = Path("control/MainFrameControl.py").read_text(encoding="utf-8")
        self.assertIn("get_machine_config_path", plc_source)
        self.assertIn("get_machine_config_path", ui_source)
        self.assertIn("strategy_name=self.strategy_name", main_source)
        self.assertNotIn('"MachineConfig.toml"', plc_source)
        self.assertNotIn('"MachineConfig.toml"', ui_source)


if __name__ == "__main__":
    unittest.main()
