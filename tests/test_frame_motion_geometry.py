import unittest
import tomllib

from model.formats.frame_by_frame.AxisFrameDataFormat import AxisData, AxisFrameData
from model.motionplan.motionutil.FrameSearchHelper import FrameSearchHelper


def populated_frame(y=1500, x_min=100, x_max=300):
    return AxisFrameData(
        FrameData=[
            AxisData(H_Axis=y, V_Axis_Max=x_max, V_Axis_Min=x_min),
        ]
    )


def empty_frame():
    return AxisFrameData(
        FrameData=[
            AxisData(H_Axis=0, V_Axis_Max=0, V_Axis_Min=0),
        ]
    )


class FrameMotionGeometryTests(unittest.TestCase):
    def setUp(self):
        self.helper = FrameSearchHelper(z_threshold=10)

    def test_window_includes_current_z_position(self):
        machine_cfg = {
            "z_position": 150,
            "out_z_front_offset": 50,
            "out_z_after_offset": 50,
        }
        window = self.helper.build_window(machine_cfg, {}, z_cur=20, frame_count=100)
        self.assertEqual((window.start, window.center, window.end), (12, 17, 22))

    def test_runtime_offsets_override_machine_defaults_and_clamp_to_frame_count(self):
        machine_cfg = {
            "z_position": 150,
            "out_z_front_offset": 50,
            "out_z_after_offset": 50,
        }
        runtime_cfg = {
            "out_z_front_offset": 20,
            "out_z_after_offset": 30,
        }
        window = self.helper.build_window(
            machine_cfg,
            runtime_cfg,
            z_cur=20,
            frame_count=20,
        )
        self.assertEqual((window.start, window.center, window.end), (15, 17, 19))

    def test_rejects_non_positive_z_threshold(self):
        for threshold in (0, -10):
            with self.subTest(threshold=threshold):
                with self.assertRaises(ValueError):
                    FrameSearchHelper(z_threshold=threshold)

    def test_spray_config_declares_stage_detection_and_x_mode(self):
        with open("model/tomls/SprayConfig.toml", "rb") as config_file:
            spray_config = tomllib.load(config_file)
        self.assertEqual(spray_config["stage_detect_frame_count"], 5)
        self.assertEqual(spray_config["frame_x_interpolation_enabled"], 1)

    def test_start_requires_exact_consecutive_front_boundary_frames(self):
        window = self.helper.create_window(10, 15, 20, frame_count=30)
        frames = [empty_frame() for _ in range(30)]
        frames[10:15] = [populated_frame() for _ in range(5)]
        self.assertTrue(self.helper.has_start_signature(frames, window, 5))

        frames[14] = empty_frame()
        self.assertFalse(self.helper.has_start_signature(frames, window, 5))
        frames[14] = populated_frame()
        frames[16] = populated_frame()
        self.assertFalse(self.helper.has_start_signature(frames, window, 5))

    def test_end_requires_exact_consecutive_rear_boundary_frames(self):
        window = self.helper.create_window(10, 15, 20, frame_count=30)
        frames = [empty_frame() for _ in range(30)]
        frames[16:21] = [populated_frame() for _ in range(5)]
        self.assertTrue(self.helper.has_end_signature(frames, window, 5))

        frames[16] = empty_frame()
        self.assertFalse(self.helper.has_end_signature(frames, window, 5))
        frames[16] = populated_frame()
        frames[14] = populated_frame()
        self.assertFalse(self.helper.has_end_signature(frames, window, 5))

    def test_full_window_data_is_neither_start_nor_end(self):
        window = self.helper.create_window(10, 15, 20, frame_count=30)
        frames = [empty_frame() for _ in range(30)]
        frames[10:21] = [populated_frame() for _ in range(11)]
        self.assertFalse(self.helper.has_start_signature(frames, window, 11))
        self.assertFalse(self.helper.has_end_signature(frames, window, 11))

    def test_invalid_detect_count_and_short_frame_list_are_safe(self):
        window = self.helper.create_window(10, 15, 20, frame_count=30)
        frames = [empty_frame() for _ in range(12)]
        self.assertFalse(self.helper.has_start_signature(frames, window, 0))
        self.assertFalse(self.helper.has_end_signature(frames, window, -1))
        self.assertFalse(self.helper.has_start_signature(frames, window, 5))
        self.assertFalse(self.helper.has_end_signature(frames, window, 5))

    def test_window_empty_requires_every_frame_to_be_empty(self):
        window = self.helper.create_window(10, 15, 20, frame_count=30)
        frames = [empty_frame() for _ in range(30)]
        self.assertTrue(self.helper.window_is_empty(frames, window))
        frames[15] = populated_frame()
        self.assertFalse(self.helper.window_is_empty(frames, window))

    def test_collects_x_range_in_y_band_across_full_z_window(self):
        frames = [empty_frame() for _ in range(30)]
        frames[9] = populated_frame(y=1400, x_min=10, x_max=1000)
        frames[12] = populated_frame(y=1400, x_min=500, x_max=700)
        frames[18] = populated_frame(y=1450, x_min=100, x_max=900)
        frames[19] = populated_frame(y=1600, x_min=20, x_max=950)
        window = self.helper.create_window(10, 15, 20, frame_count=30)
        self.assertEqual(
            self.helper.collect_x_range(frames, window, 1300, 1500),
            (100, 900),
        )

    def test_collect_x_range_returns_none_without_matching_data(self):
        frames = [empty_frame() for _ in range(30)]
        frames[15] = populated_frame(y=1600, x_min=100, x_max=900)
        window = self.helper.create_window(10, 15, 20, frame_count=30)
        self.assertEqual(
            self.helper.collect_x_range(frames, window, 1300, 1500),
            (None, None),
        )


if __name__ == "__main__":
    unittest.main()
