import os
import queue
import sys
import time
import numpy as np
import multiprocessing
from model.dataprocess.DataSplitting import DataSplitting
from model.utils.LoggerUtil import logger
from model.utils.TomlLoader import TomlLoader
from model.utils.LidarDirectionUtil import ALL_DIRECTIONS, get_active_directions
from model.utils.StrategyUtil import is_complete_workpiece_mode, is_frame_by_frame_mode, validate_strategy_name
from model.utils.WorkpieceOriginUtil import transform_points_for_origin
from model.dataprocess.frame_by_frame.BuildSideFrame import build_side_frame


# 调试数据处理
class DataProcessWorker(multiprocessing.Process):
    DIRECTIONS = ALL_DIRECTIONS

    def __init__(self, machine_data_queue, pulse_queue, viz_queue, data_paths, data_name=None, strategy_name="frame_by_frame", config_dir=None):
        super().__init__()
        self.pulse_queue = pulse_queue
        self.viz_queue = viz_queue
        self.machine_data_queue = machine_data_queue
        self.data_paths = data_paths
        self.data_name = data_name
        self.strategy_name = validate_strategy_name(strategy_name)
        self.config_dir = config_dir or (os.getcwd() + "\\model\\tomls")
        # 添加标准输出重定向
        self.stdout = sys.stdout
        self.stderr = sys.stderr

        self.data_split = DataSplitting()
        self.process_config = TomlLoader.load(f"{self.config_dir}\\ProcessConfig.toml")
        self.read_data_config = TomlLoader.load(f"{self.config_dir}\\ReadDataConfig.toml")
        self.system_config = TomlLoader.load(f"{self.config_dir}\\SystemConfig.toml")
        self.draw_type = int(self.read_data_config.get("draw_type", 1))
        self.directions = tuple(get_active_directions(self.system_config))
        # 存储所有帧数据
        self.all_frames = {}
        self.current_frame_index = 0
        self.max_frame_index = 0
        self.current_pulse = 0
        self.current_fifo = 0
        self.max_fifo = self._get_max_chaincountcm()
        self.accum = {direction: [] for direction in self.directions}
        self.lidar_status = 0

    def _get_max_chaincountcm(self):
        """基于 pulse 重置阈值推导 chaincountcm 的最大值。"""
        max_pulse = float(self.read_data_config.get("max_pulse", 160000) or 160000)
        pulse_to_mm = float(self.read_data_config.get("pulse_to_mm", 1) or 1)
        return max(1, int(round(max_pulse / pulse_to_mm / 10)))

    def run(self):
        # 重定向输出
        if hasattr(self, 'stdout'):
            sys.stdout = self.stdout
        if hasattr(self, 'stderr'):
            sys.stderr = self.stderr

        if is_frame_by_frame_mode(self.strategy_name):
            # 预先加载所有帧数据
            self._load_all_frames()

            # 主处理循环
            self._process_frames()
            return

        if is_complete_workpiece_mode(self.strategy_name):
            self._process_complete_workpieces()
            return

        logger.warning(f"Unsupported debug strategy: {self.strategy_name}")

    def _load_all_frames(self):
        """加载 frame_by_frame 调试所需的全部点云与分帧数据"""
        if not self.data_name:
            logger.warning("data_name is empty, skip frame_by_frame debug loading")
            return

        combined_path = os.path.join(self.data_paths, f"combined_{self.data_name}")
        all_xyz_data = self._safe_load_points(combined_path)
        viz_data = {
            'points': transform_points_for_origin(all_xyz_data[:, :3], self.read_data_config),
            'boxes': None
        }
        self.viz_queue.put(viz_data, block=False)
        logger.info(f"Loaded combined point cloud for visualization: {combined_path}, points={all_xyz_data.shape[0]}")

        for direction in self.directions:
            path = os.path.join(self.data_paths, f"{direction}_{self.data_name}")
            self.all_frames[direction] = self._load_direction_frames(path)
            self.max_frame_index = max(self.max_frame_index, len(self.all_frames[direction]) - 1)

        logger.info(
            "Loaded frame counts: "
            + ", ".join(f"{direction}={len(self.all_frames[direction])}" for direction in self.directions)
            + f". Max frame index: {self.max_frame_index}"
        )

    def _safe_load_points(self, path):
        """安全加载点云文件，缺失时返回空点云。"""
        if not os.path.exists(path):
            logger.warning(f"Point cloud file not found: {path}")
            return np.empty((0, 3), dtype=float)

        data = np.loadtxt(path)
        data = np.asarray(data, dtype=float)
        if data.size == 0:
            return np.empty((0, 3), dtype=float)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        if data.shape[1] < 3:
            padded = np.zeros((data.shape[0], 3), dtype=float)
            padded[:, :data.shape[1]] = data
            data = padded
        return data[:, :3]

    def _load_direction_frames(self, path):
        """按 Z 轴分帧加载单个方向点云。"""
        data = self._safe_load_points(path)
        logger.info(f"Loading data from: {path}, points={data.shape[0]}")

        if data.size == 0:
            return []

        z_threshold = int(self.process_config.get("z_threshold", 10))
        y_threshold = int(self.process_config.get("y_threshold", 10))
        z_groups = self.data_split.AxisSorting_zyx(data[:, :3], z_threshold, y_threshold)

        merged_z_groups = []
        for z_layer in z_groups:
            if z_layer:
                merged_points = np.vstack(z_layer) if len(z_layer) > 1 else z_layer[0]
                merged_z_groups.append(merged_points)
            else:
                merged_z_groups.append(np.empty((0, 3), dtype=float))
        return merged_z_groups

    def _process_frames(self):
        """处理所有帧数据，每收到一个脉冲处理一帧"""
        last_fifo = None

        while True:
            try:
                # 获取脉冲数据
                current_pulse, current_fifo = self._update_pulse_data()

                if last_fifo is None:
                    # 初始化last_fifo为当前fifo
                    last_fifo = current_fifo
                    logger.info(f"Initial FIFO set to {last_fifo}")
                    continue

                # 检查 fifo 是否前进，支持 chaincountcm 随 pulse 回绕后的 max -> 0
                repeat_count = self._get_fifo_step_delta(last_fifo, current_fifo)
                if repeat_count > 0:
                    last_fifo = current_fifo

                    # 无论是否还有真实数据，只要 FIFO 前进都发送 1 个包；跳变多少帧由 repeat_count 表示
                    self._process_current_frame(real_fifo_key=current_fifo, repeat_count=repeat_count)
                    self.current_frame_index += repeat_count
                    if self.current_frame_index > self.max_frame_index:
                        logger.info("All frames processed. Continue sending zero frames while FIFO increments...")

            except Exception as e:
                logger.error(f"Error in frame processing: {str(e)}")
                time.sleep(1)

    def _get_fifo_step_delta(self, last_fifo, current_fifo):
        """计算 FIFO 前进了多少步，支持 max_fifo -> 0 的正常回绕。"""
        ring_size = self.max_fifo + 1
        last_fifo = int(last_fifo) % ring_size
        current_fifo = int(current_fifo) % ring_size
        forward_gap = (current_fifo - last_fifo) % ring_size

        if forward_gap == 0:
            return 0

        if forward_gap <= ring_size // 2:
            return forward_gap

        return 0

    def _update_pulse_data(self):
        """从队列获取脉冲数据"""
        latest_pulse = self.current_pulse
        latest_fifo = self.current_fifo
        while not self.pulse_queue.empty():
            try:
                pulse_data = self.pulse_queue.get_nowait()
                latest_pulse = pulse_data['pulse']
                latest_fifo = pulse_data['fifo']
                logger.info(f"latest_pulse is {latest_fifo}")
            except queue.Empty:
                break
        if latest_pulse != self.current_pulse:
            self.current_pulse = latest_pulse
        if latest_fifo != self.current_fifo:
            self.current_fifo = latest_fifo
        return self.current_pulse, self.current_fifo

    def _process_current_frame(self, real_fifo_key, repeat_count):
        """处理当前帧数据：按 FIFO 前进打包四个方向的 1 个数据包发送给 PLC。"""

        # 取出当前帧四个方向点云（xyz）
        direction_data = {}
        for direction in self.directions:
            if self.current_frame_index < len(self.all_frames[direction]):
                direction_data[direction] = self.all_frames[direction][self.current_frame_index]
            else:
                direction_data[direction] = np.empty((0, 3), dtype=float)

        logger.info(f"Accumulating frame {self.current_frame_index}...")

        for direction, points in direction_data.items():
            if points.size > 0:
                self.accum[direction].append(points)

        frame_packet = {
            "fifo": real_fifo_key,
            "repeat_count": int(repeat_count),
            "lidar_status": int(getattr(self, "lidar_status", 0) or 0),
        }
        for direction in self.directions:
            frame_packet[direction] = build_side_frame(self.accum, direction, self.process_config)

        # 清除累积器（开启下一轮）
        self.accum = {direction: [] for direction in self.directions}

        # 发送给 PLC 进程
        self.machine_data_queue.put(frame_packet)

        logger.info(
            f"Sent FIFO {real_fifo_key}, repeat_count={repeat_count}, lidar_status={frame_packet['lidar_status']}: "
            f"{', '.join(self.directions)} frames to PLC"
        )

    def _process_complete_workpieces(self):
        """完整工件调试：遍历文件夹中的 combined_ 点云，模拟采数进程发送原始数据。"""
        combined_files = sorted(
            file_name for file_name in os.listdir(self.data_paths)
            if file_name.startswith("combined_") and file_name.lower().endswith(".txt")
        )

        if not combined_files:
            logger.warning(f"No combined_ files found in directory: {self.data_paths}")
            return

        for file_name in combined_files:
            file_path = os.path.join(self.data_paths, file_name)
            points = self._safe_load_points(file_path)
            if points.size == 0:
                logger.warning(f"Empty combined_ point cloud skipped: {file_path}")
                continue

            raw_data = {
                "lidar_status": 0,
                "translate_data_origin": 1,
                "all_data": points[:, :3],
                "all_stop_pulse": 0,
            }
            self.machine_data_queue.put(raw_data)
            logger.info(f"Sent complete workpiece raw data: {file_name}, points={points.shape[0]}")

            if self.draw_type != 2:
                self._push_visualization_data(points[:, :3])

            time.sleep(15000000)  # TODO 增加等待时间

    def _push_visualization_data(self, points):
        points_array = np.asarray(points, dtype=float)
        if points_array.ndim == 1:
            points_array = points_array.reshape(1, -1)
        if points_array.shape[1] < 3:
            return

        viz_data = {
            "points": transform_points_for_origin(points_array[:, :3], self.read_data_config),
            "boxes": None,
        }
        try:
            self.viz_queue.put(viz_data, block=False)
            logger.info("Debug visualization data sent to queue")
        except Exception as e:
            logger.warning(f"Failed to send debug visualization data: {str(e)}")
