import numpy as np
from model.utils.LidarDirectionUtil import SPLIT_BOTTOM_DIRECTIONS


class LidarDirectionState:
    """管理每个激光雷达方向的状态"""

    def __init__(self, direction, config):
        self.direction = direction
        self.config = config
        self.scanning_started = False
        self.has_scanned = False
        self.start_pulse = None
        self.stop_pulse: int | None = None
        self.frame_counts = []
        self.xyz_data = np.empty((0, 3))
        self.stop_reason: str | None = None        # "points" / "max_length" / "linked"
        self.is_link_anchor = False
        self.last_diff_origin_filtered = np.empty((0, 3))
        self.last_same_origin_filtered = np.empty((0, 3))
        # 方向特定阈值
        threshold_key = f"{direction}_scan_points_threshold"
        if direction in SPLIT_BOTTOM_DIRECTIONS and threshold_key not in config:
            threshold_key = "down_scan_points_threshold"
        self.points_threshold = config[threshold_key]

    def should_start_scanning(self, frame_counts):
        """判断是否应该开始扫描"""
        return len(frame_counts) == 5 and all(c >= self.points_threshold for c in frame_counts)

    def should_stop_scanning(self, frame_counts):
        """判断是否应该停止扫描"""
        return len(frame_counts) == 5 and all(c < self.points_threshold for c in frame_counts)

    def update_frame_count(self, frame_count):
        """更新帧计数"""
        self.frame_counts.append(frame_count)
        if len(self.frame_counts) > 5:
            self.frame_counts.pop(0)

    def reset(self):
        """重置状态"""
        self.scanning_started = False
        self.has_scanned = False
        self.start_pulse = None
        self.stop_pulse = None
        self.frame_counts = []
        self.xyz_data = np.empty((0, 3))
        self.stop_reason = None
        self.is_link_anchor = False
        self.last_diff_origin_filtered = np.empty((0, 3))
        self.last_same_origin_filtered = np.empty((0, 3))
