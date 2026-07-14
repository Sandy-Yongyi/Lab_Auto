import unittest

from model.motionplan.motionutil.FrameXMotionHelper import FrameXMotionHelper


class FrameXMotionHelperTests(unittest.TestCase):
    def setUp(self):
        self.helper = FrameXMotionHelper()

    def test_static_search_y_range_uses_configured_motion_range(self):
        self.assertEqual(
            self.helper.build_static_search_y_range(
                origin_pos=1200,
                y_move_min=10,
                y_move_max=400,
                out_down_y_offset=100,
                out_up_y_offset=220,
            ),
            (1110, 1820),
        )

    def test_dynamic_search_y_range_uses_current_y_position(self):
        self.assertEqual(
            self.helper.build_dynamic_search_y_range(
                origin_pos=1200,
                y_cur=150,
                out_down_y_offset=100,
                out_up_y_offset=220,
            ),
            (1250, 1570),
        )

    def test_aggregates_only_complete_static_x_ranges(self):
        self.assertEqual(
            self.helper.aggregate_static_x_range(
                [(500, 700), (None, None), (100, 900), (200, None)]
            ),
            (100, 900),
        )
        self.assertEqual(
            self.helper.aggregate_static_x_range([(None, None), (100, None)]),
            (None, None),
        )
        with self.assertRaises(ValueError):
            self.helper.aggregate_static_x_range([(900, 100)])

    def test_interpolation_speed_uses_final_target_delta(self):
        speed = self.helper.calculate_interpolation_speed(
            previous_y=0,
            current_y=100,
            previous_target=500,
            current_target=100,
            y_speed=100,
            max_speed=500,
            initial_speed=200,
        )
        self.assertEqual(speed, 400)

    def test_initial_interpolation_uses_position_speed_and_limit(self):
        self.assertEqual(
            self.helper.calculate_interpolation_speed(
                previous_y=None,
                current_y=100,
                previous_target=None,
                current_target=200,
                y_speed=100,
                max_speed=300,
                initial_speed=500,
            ),
            300,
        )

    def test_interpolation_speed_is_capped_and_uses_absolute_y_speed(self):
        self.assertEqual(
            self.helper.calculate_interpolation_speed(0, 10, 500, 100, -100, 300, 200),
            300,
        )

    def test_interpolation_speed_is_zero_without_y_or_x_distance(self):
        self.assertEqual(
            self.helper.calculate_interpolation_speed(10, 10, 500, 100, 100, 500, 200),
            0,
        )
        self.assertEqual(
            self.helper.calculate_interpolation_speed(0, 100, 500, 500, 100, 500, 200),
            0,
        )

    def test_final_target_subtracts_position_and_slow_offset(self):
        self.assertEqual(
            self.helper.build_final_x_target(800, 200, 100, 0, 1000),
            500,
        )

    def test_final_target_is_clamped_and_rejects_invalid_limits(self):
        self.assertEqual(self.helper.build_final_x_target(100, 200, 50, 0, 1000), 0)
        self.assertEqual(self.helper.build_final_x_target(2000, 100, 50, 0, 1000), 1000)
        with self.assertRaises(ValueError):
            self.helper.build_final_x_target(800, 200, 100, 1000, 0)

    def test_slow_offset_follows_front_middle_after_profile(self):
        self.assertEqual(
            self.helper.resolve_slow_offset(125, 100, 150, 50, 50, 200),
            100,
        )
        self.assertEqual(
            self.helper.resolve_slow_offset(160, 140, 150, 50, 50, 200),
            200,
        )
        self.assertEqual(
            self.helper.resolve_slow_offset(160, 175, 150, 50, 50, 200),
            100,
        )

    def test_slow_offset_matches_original_integer_truncation(self):
        self.assertEqual(
            self.helper.resolve_slow_offset(125, 100, 150, 60, 50, 200),
            116,
        )

    def test_slow_offset_handles_zero_offsets_and_clamps_progress(self):
        self.assertEqual(self.helper.resolve_slow_offset(125, 100, 150, 0, 50, 200), 200)
        self.assertEqual(self.helper.resolve_slow_offset(160, 175, 150, 50, 0, 200), 0)
        self.assertEqual(self.helper.resolve_slow_offset(0, 0, 150, 50, 50, 200), 0)
        self.assertEqual(self.helper.resolve_slow_offset(160, 300, 150, 50, 50, 200), 0)
        self.assertEqual(self.helper.resolve_slow_offset(160, 140, 150, 50, 50, -20), 0)


if __name__ == "__main__":
    unittest.main()
