import queue
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from control.DataProcessWorker import DataProcessWorker
from model.dataprocess.DataSplitting import DataSplitting


class DataProcessWorkerDebugTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.data_dir = Path(self.temp_dir.name)

    def tearDown(self):
        self.temp_dir.cleanup()

    def build_worker(self, translate_data_origin, strategy="frame_by_frame", data_name="sample.txt"):
        worker = DataProcessWorker.__new__(DataProcessWorker)
        worker.data_paths = str(self.data_dir)
        worker.data_name = data_name
        worker.strategy_name = strategy
        worker.read_data_config = {
            "translate_data_origin": translate_data_origin,
            "combined_x_min": 0,
            "combined_x_max": 2000,
            "combined_y_min": 1000,
            "combined_y_max": 2000,
            "x_threshold": 10,
            "y_threshold": 10,
            "z_threshold": 10,
            "max_pulse": 1000,
            "pulse_to_mm": 1,
            "draw_type": 2,
        }
        worker.process_config = dict(worker.read_data_config)
        worker.translate_data_origin = translate_data_origin
        worker.draw_type = 2
        worker.directions = ("left", "right")
        worker.data_split = DataSplitting()
        worker.all_frames = {}
        worker.current_frame_index = 0
        worker.max_frame_index = -1
        worker.current_pulse = 0
        worker.current_fifo = 0
        worker.max_fifo = 100
        worker.accum = {direction: [] for direction in worker.directions}
        worker.lidar_status = 0
        worker.machine_data_queue = queue.Queue()
        worker.pulse_queue = queue.Queue()
        worker.viz_queue = queue.Queue()
        return worker

    def write_points(self, file_name, rows):
        np.savetxt(self.data_dir / file_name, np.asarray(rows, dtype=float))

    def test_frame_mode_same_origin_uses_combined_file_for_all_directions(self):
        self.write_points("combined_sample.txt", [(111, 1200, 0), (222, 1200, 10)])
        self.write_points("left_sample.txt", [(333, 1200, 0)])
        self.write_points("right_sample.txt", [(444, 1200, 0)])
        worker = self.build_worker(translate_data_origin=1)

        worker._load_all_frames()

        self.assertEqual(worker.all_frames["left"][0][0, 0], 111)
        self.assertEqual(worker.all_frames["right"][0][0, 0], 111)
        self.assertEqual(len(worker.all_frames["left"]), 2)

    def test_frame_mode_different_origins_use_each_direction_file(self):
        self.write_points("combined_sample.txt", [(111, 1200, 0)])
        self.write_points("left_sample.txt", [(333, 1200, 0)])
        self.write_points("right_sample.txt", [(444, 1200, 0)])
        worker = self.build_worker(translate_data_origin=2)

        worker._load_all_frames()

        self.assertEqual(worker.all_frames["left"][0][0, 0], 333)
        self.assertEqual(worker.all_frames["right"][0][0, 0], 444)

    def test_frame_loading_preserves_empty_z_bins(self):
        self.write_points("left_sample.txt", [(100, 1200, 0), (200, 1200, 20)])
        self.write_points("right_sample.txt", [(300, 1200, 0), (400, 1200, 20)])
        worker = self.build_worker(translate_data_origin=2)

        worker._load_all_frames()

        self.assertEqual(len(worker.all_frames["left"]), 3)
        self.assertEqual(worker.all_frames["left"][1].shape, (0, 3))
        self.assertEqual(worker.all_frames["left"][2][0, 0], 200)

    def test_fifo_must_advance_and_each_step_sends_the_next_frame(self):
        worker = self.build_worker(translate_data_origin=2)
        worker.all_frames = {
            "left": [
                np.asarray([(100, 1200, 0)], dtype=float),
                np.asarray([(200, 1200, 10)], dtype=float),
            ],
            "right": [
                np.asarray([(300, 1200, 0)], dtype=float),
                np.asarray([(400, 1200, 10)], dtype=float),
            ],
        }

        sent_count = worker._process_fifo_advance(last_fifo=0, current_fifo=0)
        self.assertEqual(sent_count, 0)
        self.assertTrue(worker.machine_data_queue.empty())

        sent_count = worker._process_fifo_advance(last_fifo=0, current_fifo=2)
        self.assertEqual(sent_count, 2)
        first_packet = worker.machine_data_queue.get_nowait()
        second_packet = worker.machine_data_queue.get_nowait()
        self.assertEqual((first_packet["fifo"], second_packet["fifo"]), (1, 2))
        self.assertEqual((first_packet["repeat_count"], second_packet["repeat_count"]), (1, 1))
        self.assertEqual(self.get_first_x_min(first_packet["left"]), 100)
        self.assertEqual(self.get_first_x_min(second_packet["left"]), 200)
        self.assertEqual(worker.current_frame_index, 2)

    def test_complete_mode_same_origin_sends_combined_data_once(self):
        self.write_points("combined_sample.txt", [(111, 1200, 0)])
        self.write_points("left_sample.txt", [(333, 1200, 0)])
        self.write_points("right_sample.txt", [(444, 1200, 0)])
        worker = self.build_worker(
            translate_data_origin=1,
            strategy="complete_workpiece",
        )

        with patch("control.DataProcessWorker.time.sleep", return_value=None):
            worker._process_complete_workpieces()

        raw_data = worker.machine_data_queue.get_nowait()
        self.assertEqual(raw_data["translate_data_origin"], 1)
        self.assertEqual(raw_data["all_data"][0, 0], 111)
        self.assertNotIn("left_data", raw_data)
        self.assertNotIn("right_data", raw_data)
        self.assertIn("left_stop_pulse", raw_data)
        self.assertIn("right_stop_pulse", raw_data)
        self.assertTrue(worker.machine_data_queue.empty())

    def test_complete_mode_different_origins_sends_each_direction(self):
        self.write_points("combined_sample.txt", [(111, 1200, 0)])
        self.write_points("left_sample.txt", [(333, 1200, 0)])
        self.write_points("right_sample.txt", [(444, 1200, 0)])
        worker = self.build_worker(
            translate_data_origin=2,
            strategy="complete_workpiece",
        )

        with patch("control.DataProcessWorker.time.sleep", return_value=None):
            worker._process_complete_workpieces()

        raw_data = worker.machine_data_queue.get_nowait()
        self.assertEqual(raw_data["translate_data_origin"], 2)
        self.assertNotIn("all_data", raw_data)
        self.assertEqual(raw_data["left_data"][0, 0], 333)
        self.assertEqual(raw_data["right_data"][0, 0], 444)

    def test_complete_mode_without_name_groups_files_by_shared_suffix(self):
        self.write_points("left_first.txt", [(101, 1200, 0)])
        self.write_points("right_first.txt", [(201, 1200, 0)])
        self.write_points("left_second.txt", [(102, 1200, 0)])
        self.write_points("right_second.txt", [(202, 1200, 0)])
        worker = self.build_worker(
            translate_data_origin=2,
            strategy="complete_workpiece",
            data_name=None,
        )

        with patch("control.DataProcessWorker.time.sleep", return_value=None):
            worker._process_complete_workpieces()

        first_packet = worker.machine_data_queue.get_nowait()
        second_packet = worker.machine_data_queue.get_nowait()
        self.assertEqual(first_packet["left_data"][0, 0], 101)
        self.assertEqual(first_packet["right_data"][0, 0], 201)
        self.assertEqual(second_packet["left_data"][0, 0], 102)
        self.assertEqual(second_packet["right_data"][0, 0], 202)

    @staticmethod
    def get_first_x_min(axis_frame):
        return next(
            row.V_Axis_Min
            for row in axis_frame.FrameData
            if row.H_Axis not in (None, 0)
        )


if __name__ == "__main__":
    unittest.main()
