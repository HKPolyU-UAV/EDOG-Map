from __future__ import annotations

import numpy as np
from pystream import Stage
from pipeline.data import AppData


class SensorReaderStage(Stage):

    def __init__(self, kitti_data, crop_ratio_w, crop_ratio_h):
        self.kitti_data = kitti_data

    def __call__(self, data: AppData):
        if data.input_data.input_image is None:
            data.input_data.input_image = np.array([])
        if data.input_data.lidar_in_cam is None:
            data.input_data.lidar_in_cam = np.array([])
        if data.input_data.lidar_2d is None:
            data.input_data.lidar_2d = np.array([])
        return data

    def cleanup(self):
        pass
