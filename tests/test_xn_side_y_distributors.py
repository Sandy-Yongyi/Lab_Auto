import importlib
import tomllib
import unittest
from pathlib import Path

from model.formats.complete_workpiece.BlockDataFormat import (
    BlockData,
    InsideData,
    OutsideData,
    SubInsideData,
)


SHARED_MODULE = (
    "model.dataprocess.complete_workpiece.gun_distributors."
    "XNSharedYGunDistributor"
)
INDEPENDENT_MODULE = (
    "model.dataprocess.complete_workpiece.gun_distributors."
    "XNIndependentYGunDistributor"
)


def _load_class(module_name, class_name):
    module = importlib.import_module(module_name)
    return getattr(module, class_name)


def _machine_config():
    return {
        "sn": 1,
        "type": "xn_side",
        "spray_num": 6,
        "origin_pos": [100, 400, 700, 1000, 1300, 1600],
        "out_up_y_offset": 50,
        "out_down_y_offset": 50,
        "in_up_y_offset": 50,
        "in_down_y_offset": 50,
        "max_limit_pos": [900, 1000, 1000, 180],
    }


class XNSideYDistributorTests(unittest.TestCase):
    def test_distributor_modules_use_clear_shared_and_independent_names(self):
        shared_class = _load_class(SHARED_MODULE, "XNSharedYGunDistributor")
        independent_class = _load_class(
            INDEPENDENT_MODULE,
            "XNIndependentYGunDistributor",
        )

        self.assertEqual(shared_class.__name__, "XNSharedYGunDistributor")
        self.assertEqual(
            independent_class.__name__,
            "XNIndependentYGunDistributor",
        )

    def test_spray_config_declares_y_mode_without_independent_y_limit(self):
        with Path("model/tomls/SprayConfig.toml").open("rb") as config_file:
            spray_config = tomllib.load(config_file)

        self.assertIn(spray_config["xn_side_y_mode"], (0, 1))
        self.assertNotIn("spray_y_max", spray_config)

    def test_independent_outside_keeps_gun_count_when_no_lower_candidate_exists(self):
        distributor_class = _load_class(
            INDEPENDENT_MODULE,
            "XNIndependentYGunDistributor",
        )

        selected = distributor_class._select_outside_guns(
            indexed_origins=[
                (0, 1120),
                (1, 1420),
                (2, 1720),
                (3, 2020),
                (4, 2320),
                (5, 2620),
            ],
            target_y_min=1000,
            target_y_max=3000,
            gun_distance=300,
            gun_num=3,
        )

        self.assertEqual(selected, [(0, 1120), (1, 1420), (2, 1720)])

    def test_independent_outside_assigns_an_individual_range_to_each_gun(self):
        distributor_class = _load_class(
            INDEPENDENT_MODULE,
            "XNIndependentYGunDistributor",
        )
        distributor = distributor_class(
            spray_config={"min_recip_distance": 60},
        )
        blockdata = BlockData(
            outside_data=[
                OutsideData(outside_y_min=200, outside_y_max=1500),
            ],
            inside_data=[],
        )

        groups = distributor.distribute(
            blockdata,
            _machine_config(),
            gun_distance=300,
        )

        outside_guns = groups[0].gundata_list
        self.assertEqual(groups[0].group_type, "outside")
        self.assertEqual(
            [
                (
                    gun.gun_y_enable,
                    gun.gun_y_downer,
                    gun.gun_y_upper,
                    gun.gun_r_angle,
                )
                for gun in outside_guns
            ],
            [
                (1, 50, 400, 0),
                (1, 100, 450, 0),
                (1, 150, 500, 0),
                (1, 200, 550, 0),
                (0, 550, 550, 0),
                (0, 550, 550, 0),
            ],
        )

    def test_independent_outside_reduces_ranges_at_y_limit(self):
        distributor_class = _load_class(
            INDEPENDENT_MODULE,
            "XNIndependentYGunDistributor",
        )
        distributor = distributor_class(spray_config={"min_recip_distance": 60})

        outside_guns = distributor._build_outside_guns(
            out=OutsideData(outside_y_min=200, outside_y_max=1500),
            origin_pos=[100, 400, 700, 1000, 1300, 1600],
            axis_num=6,
            up_y_offset=0,
            down_y_offset=0,
            y_limit=430,
            scan_span=300,
        )

        self.assertEqual(
            [(gun.gun_y_downer, gun.gun_y_upper) for gun in outside_guns],
            [
                (100, 407),
                (107, 414),
                (114, 421),
                (121, 428),
                (430, 430),
                (430, 430),
            ],
        )
    def test_independent_inside_builds_one_group_for_each_inside_column(self):
        distributor_class = _load_class(
            INDEPENDENT_MODULE,
            "XNIndependentYGunDistributor",
        )
        distributor = distributor_class(
            spray_config={"min_recip_distance": 60},
        )
        blockdata = BlockData(
            outside_data=[
                OutsideData(outside_y_min=200, outside_y_max=1500),
            ],
            inside_data=[
                InsideData(
                    inside_id=10,
                    subinside_datalist=[
                        SubInsideData(
                            subinside_y_min=100,
                            subinside_y_max=800,
                        ),
                        SubInsideData(
                            subinside_y_min=1000,
                            subinside_y_max=1600,
                        ),
                    ],
                ),
                InsideData(
                    inside_id=11,
                    subinside_datalist=[
                        SubInsideData(
                            subinside_y_min=800,
                            subinside_y_max=1400,
                        ),
                    ],
                ),
            ],
        )

        groups = distributor.distribute(
            blockdata,
            _machine_config(),
            gun_distance=300,
        )

        self.assertEqual(
            [(group.group_type, group.group_id) for group in groups],
            [("outside", 0), ("inside", 10), ("inside", 11)],
        )
        first_inside = groups[1].gundata_list
        self.assertEqual(
            [
                (
                    gun.gun_y_enable,
                    gun.gun_y_downer,
                    gun.gun_y_upper,
                )
                for gun in first_inside
            ],
            [
                (1, 50, 350),
                (1, 350, 350),
                (1, 350, 850),
                (0, 850, 850),
                (0, 850, 850),
                (0, 850, 850),
            ],
        )
        second_inside = groups[2].gundata_list
        self.assertEqual(
            [
                (
                    gun.gun_y_enable,
                    gun.gun_y_downer,
                    gun.gun_y_upper,
                )
                for gun in second_inside
            ],
            [
                (0, 0, 0),
                (0, 0, 0),
                (1, 150, 650),
                (0, 650, 650),
                (0, 650, 650),
                (0, 650, 650),
            ],
        )

    def test_independent_inside_short_block_stays_at_center(self):
        distributor_class = _load_class(INDEPENDENT_MODULE, "XNIndependentYGunDistributor")
        distributor = distributor_class(spray_config={"min_recip_distance": 60})

        guns = distributor._build_inside_guns(
            inside_blocks=[SubInsideData(subinside_y_min=200, subinside_y_max=240)],
            origin_pos=[100, 400, 700],
            axis_num=3,
            up_y_offset=0,
            down_y_offset=0,
            min_recip_distance=60,
            y_limit=1000,
            scan_span=300,
        )

        self.assertEqual(
            [(gun.gun_y_enable, gun.gun_y_downer, gun.gun_y_upper) for gun in guns],
            [(1, 120, 120), (0, 120, 120), (0, 120, 120)],
        )

    def test_independent_inside_prevents_overlap_when_move_ranges_differ(self):
        distributor_class = _load_class(INDEPENDENT_MODULE, "XNIndependentYGunDistributor")
        distributor = distributor_class(spray_config={"min_recip_distance": 0})

        guns = distributor._build_inside_guns(
            inside_blocks=[
                SubInsideData(subinside_y_min=243, subinside_y_max=386),
                SubInsideData(subinside_y_min=333, subinside_y_max=830),
                SubInsideData(subinside_y_min=763, subinside_y_max=900),
            ],
            origin_pos=[0, 0, 0],
            axis_num=3,
            up_y_offset=0,
            down_y_offset=0,
            min_recip_distance=0,
            y_limit=900,
            scan_span=1000,
        )

        self.assertEqual(
            [(gun.gun_y_downer, gun.gun_y_upper) for gun in guns],
            [(243, 386), (386, 830), (830, 900)],
        )

    def test_independent_inside_keeps_monotonic_ranges_when_move_ranges_match(self):
        distributor_class = _load_class(INDEPENDENT_MODULE, "XNIndependentYGunDistributor")
        distributor = distributor_class(spray_config={"min_recip_distance": 0})

        guns = distributor._build_inside_guns(
            inside_blocks=[
                SubInsideData(subinside_y_min=50, subinside_y_max=150),
                SubInsideData(subinside_y_min=100, subinside_y_max=200),
                SubInsideData(subinside_y_min=150, subinside_y_max=250),
            ],
            origin_pos=[0, 0, 0],
            axis_num=3,
            up_y_offset=0,
            down_y_offset=0,
            min_recip_distance=0,
            y_limit=1000,
            scan_span=1000,
        )

        self.assertEqual(
            [(gun.gun_y_downer, gun.gun_y_upper) for gun in guns],
            [(50, 150), (100, 200), (150, 250)],
        )

    def test_independent_inside_reserves_guns_for_later_blocks(self):
        distributor_class = _load_class(INDEPENDENT_MODULE, "XNIndependentYGunDistributor")
        distributor = distributor_class(spray_config={"min_recip_distance": 60})

        guns = distributor._build_inside_guns(
            inside_blocks=[
                SubInsideData(subinside_y_min=800, subinside_y_max=1100),
                SubInsideData(subinside_y_min=1200, subinside_y_max=1800),
                SubInsideData(subinside_y_min=1900, subinside_y_max=2500),
            ],
            origin_pos=[100, 400, 700, 1000, 1300, 1600],
            axis_num=6,
            up_y_offset=0,
            down_y_offset=0,
            min_recip_distance=60,
            y_limit=1000,
            scan_span=300,
        )

        self.assertEqual(
            [(gun.gun_y_enable, gun.gun_y_downer, gun.gun_y_upper) for gun in guns],
            [(0, 0, 0), (1, 400, 700), (1, 500, 800), (1, 500, 800), (1, 600, 900), (1, 600, 900)],
        )

    def test_independent_inside_prioritizes_lower_blocks_when_guns_are_insufficient(self):
        distributor_class = _load_class(INDEPENDENT_MODULE, "XNIndependentYGunDistributor")
        distributor = distributor_class(spray_config={"min_recip_distance": 60})

        guns = distributor._build_inside_guns(
            inside_blocks=[
                SubInsideData(subinside_y_min=100, subinside_y_max=1000),
                SubInsideData(subinside_y_min=1100, subinside_y_max=2000),
                SubInsideData(subinside_y_min=2100, subinside_y_max=2700),
            ],
            origin_pos=[100, 400, 700, 1000, 1300, 1600],
            axis_num=6,
            up_y_offset=0,
            down_y_offset=0,
            min_recip_distance=60,
            y_limit=1000,
            scan_span=300,
        )

        self.assertEqual(
            [(gun.gun_y_enable, gun.gun_y_downer, gun.gun_y_upper) for gun in guns],
            [(1, 0, 300), (1, 0, 300), (1, 0, 300), (1, 100, 400), (1, 100, 400), (1, 100, 400)],
        )

    def test_gun_distributor_routes_only_xn_side_by_y_mode(self):
        gun_distributor_class = _load_class(
            "model.dataprocess.complete_workpiece.GunDistributor",
            "GunDistributor",
        )

        class RecordingDistributor:
            def __init__(self, result):
                self.calls = 0
                self.result = result

            def distribute(self, blockdata, machine_config, gun_distance):
                self.calls += 1
                return self.result

        shared = RecordingDistributor(["shared"])
        independent = RecordingDistributor(["independent"])
        distributor = gun_distributor_class.__new__(gun_distributor_class)
        distributor.shared_side_distributor = shared
        distributor.independent_side_distributor = independent
        distributor.spray_cfg = {"xn_side_y_mode": 0}

        shared_result = distributor._build_machine_distribution(
            blockdata=BlockData(),
            machine_cfg={"type": "xn_side"},
            machine_id=1,
            is_cabinet=True,
            spray_width_distance=30,
            gun_distance=430,
        )
        distributor.spray_cfg["xn_side_y_mode"] = 1
        independent_result = distributor._build_machine_distribution(
            blockdata=BlockData(),
            machine_cfg={"type": "xn_side"},
            machine_id=1,
            is_cabinet=True,
            spray_width_distance=30,
            gun_distance=430,
        )

        self.assertEqual(shared_result.gun_groups, ["shared"])
        self.assertEqual(independent_result.gun_groups, ["independent"])
        self.assertEqual(shared.calls, 1)
        self.assertEqual(independent.calls, 1)


if __name__ == "__main__":
    unittest.main()
