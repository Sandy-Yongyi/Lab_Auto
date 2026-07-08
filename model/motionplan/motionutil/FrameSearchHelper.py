class FrameSearchHelper:
    """帧数据搜索辅助类。"""

    def __init__(self, z_threshold: int = 10):
        self.z_threshold = z_threshold

    def get_side_frames(self, machine_cfg, frame_queue_manager):
        direction = self.get_side_direction(machine_cfg)
        return frame_queue_manager.frame_stack.get(direction, [])

    def get_upper_frames(self, machine_cfg, frame_queue_manager):
        direction = self.get_upper_direction(machine_cfg)
        return frame_queue_manager.frame_stack.get(direction, [])

    def get_side_direction(self, machine_cfg):
        return "left" if machine_cfg.get("install_orietation", "left") == "left" else "right"

    def get_upper_direction(self, machine_cfg):
        return "left_upper" if machine_cfg.get("install_orietation", "left") == "left" else "right_upper"

    def get_frame_by_index(self, frames, index):
        if index < 0 or index >= len(frames):
            return None
        return frames[index]

    def iter_window_indices(self, start_idx, end_idx):
        if start_idx <= end_idx:
            return range(start_idx, end_idx + 1)
        return range(start_idx, end_idx - 1, -1)

    def row_has_data(self, row):
        if row is None:
            return False
        h_axis = getattr(row, "H_Axis", 0)
        v_max = getattr(row, "V_Axis_Max", 0)
        v_min = getattr(row, "V_Axis_Min", 0)
        return not (int(h_axis or 0) == 0 and int(v_max or 0) == 0 and int(v_min or 0) == 0)

    def frame_has_data(self, frame):
        if frame is None or not getattr(frame, "FrameData", None):
            return False
        return any(self.row_has_data(row) for row in frame.FrameData)

    def frame_has_y_in_band(self, frame, y_lower, y_upper):
        if frame is None or not getattr(frame, "FrameData", None):
            return False
        for row in frame.FrameData:
            if not self.row_has_data(row):
                continue
            y_val = int(getattr(row, "H_Axis", 0) or 0)
            if y_lower <= y_val <= y_upper:
                return True
        return False

    def scan_y_range(self, frames, start_idx, end_idx):
        y_values = []
        for idx in self.iter_window_indices(start_idx, end_idx):
            frame = self.get_frame_by_index(frames, idx)
            if frame is None or not getattr(frame, "FrameData", None):
                continue
            for row in frame.FrameData:
                if self.row_has_data(row):
                    y_values.append(int(row.H_Axis))
        if not y_values:
            return None, None
        return min(y_values), max(y_values)

    def scan_y_min(self, frames, start_idx, end_idx):
        y_min, _ = self.scan_y_range(frames, start_idx, end_idx)
        return y_min

    def collect_x_min_values(self, frames, start_idx, end_idx, y_start, y_end):
        x_values = []
        for idx in self.iter_window_indices(start_idx, end_idx):
            frame = self.get_frame_by_index(frames, idx)
            if frame is None or not getattr(frame, "FrameData", None):
                continue
            for row in frame.FrameData:
                if not self.row_has_data(row):
                    continue
                y_val = int(getattr(row, "H_Axis", 0) or 0)
                if y_start <= y_val <= y_end:
                    x_min = int(getattr(row, "V_Axis_Min", 0) or 0)
                    if x_min != 0:
                        x_values.append(x_min)
        return x_values

    def collect_x_values(self, frames, start_idx, end_idx, y_start, y_end):
        x_values = []
        for idx in self.iter_window_indices(start_idx, end_idx):
            frame = self.get_frame_by_index(frames, idx)
            if frame is None or not getattr(frame, "FrameData", None):
                continue
            for row in frame.FrameData:
                if not self.row_has_data(row):
                    continue
                y_val = int(getattr(row, "H_Axis", 0) or 0)
                if y_start <= y_val <= y_end:
                    x_min = int(getattr(row, "V_Axis_Min", 0) or 0)
                    x_max = int(getattr(row, "V_Axis_Max", 0) or 0)
                    if x_min != 0:
                        x_values.append(x_min)
                    if x_max != 0:
                        x_values.append(x_max)
        return x_values

    def scan_vertical_range_by_row_window(self, frame, start_row, end_row):
        if frame is None or not getattr(frame, "FrameData", None):
            return None, None
        if start_row > end_row:
            start_row, end_row = end_row, start_row

        row_start = max(0, int(start_row))
        row_end = min(len(frame.FrameData) - 1, int(end_row))
        if row_start > row_end:
            return None, None

        v_mins = []
        v_maxs = []
        for row_idx in range(row_start, row_end + 1):
            row = frame.FrameData[row_idx]
            if not self.row_has_data(row):
                continue
            v_min = int(getattr(row, "V_Axis_Min", 0) or 0)
            v_max = int(getattr(row, "V_Axis_Max", 0) or 0)
            if v_min != 0:
                v_mins.append(v_min)
            if v_max != 0:
                v_maxs.append(v_max)

        if not v_mins and not v_maxs:
            return None, None

        final_min = min(v_mins) if v_mins else None
        final_max = max(v_maxs) if v_maxs else None
        return final_min, final_max
