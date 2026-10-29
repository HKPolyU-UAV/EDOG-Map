"""Exercise the public mapping pipeline without ROS or external datasets.

Run from any directory with: python /path/to/EDOG-Map/scripts/smoke_test.py
This checks ordinary module behavior, not the paper's benchmark claims.
"""

from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
from pystream import Pipeline

from libs.dgm import (
    MAP_SIZE_X,
    MAP_SIZE_Y,
    generate_measurement_dgm,
    init_dgm,
    predict_dgm,
    update_dgm,
)
from pipeline.data import AppData
from pipeline.dgm_map import DGMMapStage
from pipeline.output_writer import OutputWriterStage
from pipeline.road_segmentation import RoadSegmentationStage
from pipeline.sensor_reader import SensorReaderStage


def check_mass_grid(grid):
    assert grid.shape == (MAP_SIZE_Y, MAP_SIZE_X, 3), grid.shape
    assert np.isfinite(grid).all(), "Non-finite belief masses"
    assert (grid >= -1e-6).all() and (grid <= 1.0 + 1e-6).all()
    np.testing.assert_allclose(grid.sum(axis=2), 1.0, atol=2e-6)


def check_rgb_image(image):
    assert image.shape == (MAP_SIZE_Y, MAP_SIZE_X, 3), image.shape
    assert image.dtype == np.uint8, image.dtype
    palette = np.array(
        [(123, 123, 123), (0, 0, 0), (255, 255, 255), (255, 0, 0)],
        dtype=np.uint8,
    )
    valid = np.zeros(image.shape[:2], dtype=bool)
    for color in palette:
        valid |= np.all(image == color, axis=2)
    assert valid.all(), "Unexpected visualization color"


def main():
    # Synthetic horizontal scan points; no download, ROS master, or model needed.
    points = np.array(
        [[10.0, 0.0, 1.0, 1.0], [8.0, 3.0, 1.0, 1.0],
         [4.0, -6.0, 1.0, 1.0], [-5.0, 2.0, 1.0, 1.0]],
        dtype=np.float32,
    )
    prior = init_dgm()
    check_mass_grid(prior)
    assert np.all(prior[:, :, 0] == 1.0), "Initial map must be unknown"
    assert not prior[:, :, 1:].any()

    measurement = generate_measurement_dgm(points[:, :2], prior.shape)
    check_mass_grid(measurement)
    fused, conflict = update_dgm(prior, measurement)
    check_mass_grid(fused)
    assert np.isfinite(conflict).all() and not conflict.any()
    np.testing.assert_allclose(fused, measurement, atol=2e-6)
    fused, conflict = update_dgm(fused, measurement)
    check_mass_grid(fused)
    assert not conflict.any(), "Identical evidence should not conflict"
    check_rgb_image(predict_dgm(fused, conflict))
    del prior, measurement, fused, conflict

    with tempfile.TemporaryDirectory(prefix="edog-map-smoke-") as temporary:
        pipeline = Pipeline()
        pipeline.add(SensorReaderStage(None, 4, 3))
        # An absent model is intentional: empty image input must bypass loading.
        pipeline.add(RoadSegmentationStage(str(Path(temporary) / "absent.pb"), 513))
        pipeline.add(DGMMapStage(0.15))
        pipeline.add(OutputWriterStage(temporary))
        pipeline.serialize()
        try:
            for index in range(2):
                data = AppData()
                data.metadata.data_idx = index
                data.metadata.previous_data_idx = max(index - 1, 0)
                data.input_data.lidar_raw = points
                pipeline.forward(data)
                result = pipeline.get_results()
                assert result is not None, "Serial pipeline produced no result"
                assert result.output_data.segmentation_pred.size == 0
                check_mass_grid(result.output_data.dgm)
                check_rgb_image(result.output_data.dgm_pred)
                saved = np.load(Path(temporary) / f"{index:06d}_dgm_pred.npy")
                np.testing.assert_array_equal(saved, result.output_data.dgm_pred)
        finally:
            pipeline.cleanup()

    print("PASS: mass grids, ordinary fusion, RGB rendering, and serial pipeline output")


if __name__ == "__main__":
    main()
