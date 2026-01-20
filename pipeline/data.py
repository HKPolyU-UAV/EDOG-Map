from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple
import numpy as np


def _empty_array() -> np.ndarray:
    return np.array([])


@dataclass
class Metadata:
    data_idx: int = 0
    previous_data_idx: int = 0


@dataclass
class InputData:
    input_image: np.ndarray = field(default_factory=_empty_array)
    lidar_in_cam: np.ndarray = field(default_factory=_empty_array)
    lidar_2d: np.ndarray = field(default_factory=_empty_array)
    lidar_raw: np.ndarray = field(default_factory=_empty_array)
    pose: Tuple = (0.0, 0.0, 0.0)
    previous_pose: Tuple = (0.0, 0.0, 0.0)


@dataclass
class OutputData:
    segmentation_pred: np.ndarray = field(default_factory=_empty_array)
    ogm: np.ndarray = field(default_factory=_empty_array)
    dgm: np.ndarray = field(default_factory=_empty_array)
    dynamic_mass: np.ndarray = field(default_factory=_empty_array)
    dgm_pred: np.ndarray = field(default_factory=_empty_array)


@dataclass
class AppData:
    metadata: Metadata = field(default_factory=Metadata)
    input_data: InputData = field(default_factory=InputData)
    output_data: OutputData = field(default_factory=OutputData)
