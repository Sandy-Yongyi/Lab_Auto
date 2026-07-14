import unittest
from pathlib import Path

from model.utils.StrategyUtil import strategy_name_from_code


class StrategyModeConfigTests(unittest.TestCase):
    def test_maps_supported_integer_codes(self):
        self.assertEqual(strategy_name_from_code(1), "frame_by_frame")
        self.assertEqual(strategy_name_from_code(2), "complete_workpiece")
        self.assertEqual(strategy_name_from_code(3), "continuous_bidirectional")

    def test_rejects_bool_string_and_unknown_code(self):
        for value in (True, "1", 0, 4, None):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    strategy_name_from_code(value)

    def test_mode_config_defaults_to_frame_by_frame(self):
        source = Path("model/tomls/ModeConfig.toml").read_text(encoding="utf-8")
        self.assertIn("strategy_name = 1", source)

    def test_main_frame_uses_cached_strategy(self):
        source = Path("control/MainFrameControl.py").read_text(encoding="utf-8")
        self.assertIn("self.strategy_name = strategy_name_from_code", source)
        self.assertIn("strategy_name = self.strategy_name", source)
        self.assertNotIn('validate_strategy_name("complete_workpiece")', source)


if __name__ == "__main__":
    unittest.main()
