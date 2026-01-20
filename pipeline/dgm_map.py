from __future__ import annotations

import threading
import numpy as np
from pystream import Stage
from pipeline.data import AppData

from libs.dgm import (
    init_dgm,
    shift_pose_dgm,
    generate_measurement_dgm,
    update_dgm,
    predict_dgm,
)


class DGMMapStage(Stage):
    def __init__(self, road_height_threshold):
        self.dgm = init_dgm()
        self._lock = threading.Lock()

    def __call__(self, data: AppData):
        with self._lock:
            shifted = shift_pose_dgm(self.dgm, data.input_data.previous_pose, data.input_data.pose)

            if isinstance(data.input_data.lidar_raw, np.ndarray) and data.input_data.lidar_raw.size > 0:
                pts2d = data.input_data.lidar_raw[:, :2]
            else:
                pts2d = None

            meas = generate_measurement_dgm(pts2d, shifted.shape)
            self.dgm, dynamic_mass = update_dgm(shifted, meas)

        data.output_data.dgm = self.dgm
        data.output_data.dynamic_mass = dynamic_mass
        data.output_data.dgm_pred = predict_dgm(self.dgm, dynamic_mass)
        return data

    def cleanup(self):
        pass
