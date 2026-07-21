import unittest
from types import SimpleNamespace

from model.formats.complete_workpiece.BlockDataFormat import DistribeGunData, GunGroupData
from model.motionplan.MachineAxisMap import get_axis_map
from model.motionplan.MotionReciprocate import MotionReciprocate
from model.plc.MovingFrameData import AxisData


class MotionReciprocateYModeTests(unittest.TestCase):
    def setUp(self):
        self.machine_cfg = {
            "sn": 1,
            "type": "xn_side",
            "install_orietation": "left",
            "spray_num": 3,
            "max_limit_speed": [250, 500, 500, 160],
            "min_limit_pos": [0, 0, 0, -180],
            "max_limit_pos": [900, 430, 1000, 180],
            "safe_pos": [0, 0, 0, 0],
            "outside_total_cycles": 1,
            "recip_reduce_distance": 0,
        }
        self.runtime_cfg = {
            "outside_total_cycles": 1,
            "recip_reduce_distance": 0,
            "x_pos_speed": 100,
            "x_recip_speed": 100,
            "y_recip_speed": 100,
        }
        self.gun_group = GunGroupData(
            group_type="outside",
            group_id=0,
            gundata_list=[
                DistribeGunData(gun_id=0, gun_y_enable=1, gun_y_downer=10, gun_y_upper=110, gun_r_angle=0),
                DistribeGunData(gun_id=1, gun_y_enable=1, gun_y_downer=20, gun_y_upper=220, gun_r_angle=0),
                DistribeGunData(gun_id=2, gun_y_enable=0, gun_y_downer=50, gun_y_upper=50, gun_r_angle=0),
            ],
        )
        self.plc = SimpleNamespace(
            AxisList=[AxisData() for _ in range(48)],
            ChainStatus="moving_forward",
            ChainSpeed=100,
        )
        self.axis_map = get_axis_map("xn_side", "left")
        self.motion = MotionReciprocate.__new__(MotionReciprocate)
        self.motion.tolerance = 5
        self.motion.min_recip_distance = 60
        self.motion._rect_states = {}
        self.motion._y_states = {}
        for axis_name in ("x1", "x2", "x3"):
            self._set_pos(axis_name, 100)

    def _set_mode(self, y_mode, recip_mode="2d", cycle_axis="y"):
        self.motion.spray_cfg = {
            "xn_side_y_mode": y_mode,
            "side_reciprocate_mode": recip_mode,
            "side_2d_cycle_axis": cycle_axis,
        }

    def _set_pos(self, axis_name, position):
        self.plc.AxisList[self.axis_map[axis_name]].Pos = position

    def _build_2d(self):
        return self.motion.build_side_reciprocate(
            machine_cfg=self.machine_cfg,
            runtime_cfg=self.runtime_cfg,
            plc_data=self.plc,
            gun_group=self.gun_group,
            x_min=100,
            x_max=200,
            r_angle=0,
            rect_threshold=150,
            state_key="sn1:outside",
            total_cycles_key="outside_total_cycles",
        )

    def test_shared_y_reads_y1_and_keeps_logical_y_output(self):
        self._set_mode(y_mode=0)
        self._set_pos("y1", 20)

        axis_cmds, done = self._build_2d()

        self.assertFalse(done)
        self.assertEqual(axis_cmds["y"], AxisData(Pos=20, Speed=100, Status=0))
        self.assertNotIn("y1", axis_cmds)
        self.assertNotIn("y2", axis_cmds)

    def test_independent_y_finishes_only_after_every_enabled_y_axis_completes(self):
        self._set_mode(y_mode=1)
        self._set_pos("y1", 10)
        self._set_pos("y2", 20)
        self._set_pos("y3", 50)

        axis_cmds, done = self._build_2d()
        self.assertFalse(done)
        self.assertNotIn("y", axis_cmds)
        self.assertEqual(axis_cmds["y1"], AxisData(Pos=10, Speed=100, Status=0))
        self.assertEqual(axis_cmds["y2"], AxisData(Pos=20, Speed=100, Status=0))
        self.assertEqual(axis_cmds["y3"], AxisData(Pos=50, Speed=100, Status=0))

        self._set_pos("y1", 110)
        self._set_pos("y2", 100)
        _, done = self._build_2d()
        self.assertFalse(done)

        self._set_pos("y1", 10)
        self._set_pos("y2", 220)
        axis_cmds, done = self._build_2d()
        self.assertFalse(done)
        self.assertEqual(axis_cmds["x1"].Status, 0)
        self.assertEqual(axis_cmds["x2"].Status, 1)

        self._set_pos("y1", 10)
        self._set_pos("y2", 20)
        axis_cmds, done = self._build_2d()
        self.assertTrue(done)
        self.assertEqual(axis_cmds["y1"], AxisData(Pos=10, Speed=0, Status=0))
        self.assertEqual(axis_cmds["y2"], AxisData(Pos=20, Speed=100, Status=0))
        self.assertEqual(axis_cmds["y3"], AxisData(Pos=50, Speed=100, Status=0))

    def test_independent_fixed_y_uses_x_cycles_when_y_controls_completion(self):
        self._set_mode(y_mode=1)
        self.gun_group.gundata_list[0].gun_y_downer = 50
        self.gun_group.gundata_list[0].gun_y_upper = 50
        self._set_pos("y1", 50)
        self._set_pos("y2", 20)
        self._set_pos("y3", 50)

        _, done = self._build_2d()
        self.assertFalse(done)

        self._set_pos("x1", 200)
        self._set_pos("x2", 200)
        self._set_pos("y2", 100)
        _, done = self._build_2d()
        self.assertFalse(done)

        self._set_pos("x1", 100)
        self._set_pos("x2", 100)
        axis_cmds, done = self._build_2d()
        self.assertFalse(done)
        self.assertEqual(axis_cmds["x1"].Status, 0)
        self.assertEqual(axis_cmds["x2"].Status, 1)

        self._set_pos("y2", 220)
        _, done = self._build_2d()
        self.assertFalse(done)
        self._set_pos("y2", 20)
        axis_cmds, done = self._build_2d()
        self.assertTrue(done)
        self.assertEqual(axis_cmds["x1"].Status, 0)
        self.assertEqual(axis_cmds["x2"].Status, 0)

    def test_independent_y_end_face_uses_each_gun_range(self):
        self._set_mode(y_mode=1)
        self._set_pos("y1", 10)
        self._set_pos("y2", 20)
        self._set_pos("y3", 50)

        axis_cmds = self.motion.build_side_end_face_reciprocate(
            machine_cfg=self.machine_cfg,
            runtime_cfg=self.runtime_cfg,
            plc_data=self.plc,
            gun_group=self.gun_group,
            x_target=100,
            y_min=0,
            y_max=430,
            r_angle=0,
            state_key="sn1:end_face",
            z_target=900,
            z_speed=100,
        )

        self.assertNotIn("y", axis_cmds)
        self.assertEqual(axis_cmds["y1"], AxisData(Pos=10, Speed=100, Status=0))
        self.assertEqual(axis_cmds["y2"], AxisData(Pos=20, Speed=100, Status=0))
        self.assertEqual(axis_cmds["y3"], AxisData(Pos=50, Speed=100, Status=0))

        axis_cmds = self.motion.build_side_end_face_reciprocate(
            machine_cfg=self.machine_cfg,
            runtime_cfg=self.runtime_cfg,
            plc_data=self.plc,
            gun_group=self.gun_group,
            x_target=100,
            y_min=0,
            y_max=430,
            r_angle=0,
            state_key="sn1:end_face",
            z_target=900,
            z_speed=100,
        )
        self.assertEqual(axis_cmds["y1"], AxisData(Pos=110, Speed=100, Status=0))
        self.assertEqual(axis_cmds["y2"], AxisData(Pos=220, Speed=100, Status=0))

    def test_independent_y_rect_mode_maps_each_y_range_to_same_segment(self):
        self._set_mode(y_mode=1, recip_mode="rect")
        self._set_pos("x1", 200)
        self._set_pos("x2", 200)

        _, done = self.motion.build_side_reciprocate(
            machine_cfg=self.machine_cfg,
            runtime_cfg=self.runtime_cfg,
            plc_data=self.plc,
            gun_group=self.gun_group,
            x_min=100,
            x_max=200,
            r_angle=0,
            rect_threshold=150,
            state_key="sn1:rect",
            total_cycles_key="outside_total_cycles",
        )
        self.assertFalse(done)

        axis_cmds, done = self.motion.build_side_reciprocate(
            machine_cfg=self.machine_cfg,
            runtime_cfg=self.runtime_cfg,
            plc_data=self.plc,
            gun_group=self.gun_group,
            x_min=100,
            x_max=200,
            r_angle=0,
            rect_threshold=150,
            state_key="sn1:rect",
            total_cycles_key="outside_total_cycles",
        )

        self.assertFalse(done)
        self.assertNotIn("y", axis_cmds)
        self.assertEqual(axis_cmds["y1"], AxisData(Pos=35, Speed=100, Status=0))
        self.assertEqual(axis_cmds["y2"], AxisData(Pos=70, Speed=100, Status=0))
        self.assertEqual(axis_cmds["y3"], AxisData(Pos=50, Speed=100, Status=0))


if __name__ == "__main__":
    unittest.main()
