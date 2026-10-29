# Implementation notes

This page describes the Python code currently in this repository. The
[paper](https://doi.org/10.1109/TCE.2026.3725136) describes the research system and
its experiments; its benchmark results have not been independently reproduced by
the smoke test in this repository.

## Scope of the public implementation

- `run.py` consumes synchronized ROS 1 point clouds and odometry. It uses only
  horizontal point coordinates and planar `(x, y, yaw)` poses after a height
  filter. It does not transform message frames or estimate odometry.
- `libs/dgm.py` maintains a fixed-size, robot-centered 2D array with three mass
  channels: **unknown, occupied, free**. The prior is warped with OpenCV, fused
  with the current measurement, and colored using the conflict mass. The code
  does not maintain per-cell velocity distributions.
- The map is bounded in spatial size, but `shift_pose_dgm` allocates and warps
  dense arrays. The pointer-based memory reuse described in Section III-B of the
  paper is not implemented here. Fixed map dimensions do not imply constant
  computation time with respect to the number of cells. Output files and the
  writer's timing history also grow with the number of processed frames.
- Current defaults are a 100 m map width, 0.1 m cells, and a 50 m sensing radius.
  These differ from the 0.2 m grid resolution and 10 m local update range
  described for the paper's field deployments. They must not be presented as
  the configuration used to obtain the paper's reported results.
- The ROS entry point leaves image inputs empty, so the road segmentation stage
  is a pass-through. TensorFlow and the referenced DeepLab model are unnecessary
  for that path. Legacy camera/road-filter helpers remain in the repository but
  are not an end-to-end KITTI loader or a training pipeline.
- The output writer saves RGB renderings as `.npy` and, when Matplotlib is
  available, `.png`. These files are not raw mass tensors, velocity estimates,
  ROS occupancy messages, or planner commands. Its reported throughput includes
  message arrival intervals and output I/O; it is not a mapping-only benchmark.
- Dataset loaders, ground-truth annotations, evaluation scripts, onboard launch
  files, and planning integration for the paper's experiments are not included.

The inverse sensor model marks angular bins without returns as free out to the
configured range. This assumption needs review for a sensor with limited field
of view, missing beams, occlusion, or aggressive point filtering. Point clouds
must already be expressed in the body frame consistent with the odometry; the
`use_forward_right` option changes the pose signs only, not the cloud coordinates.

## Two reproducible edge cases to resolve

The following examples use the existing code without modification. Run them from
the repository root after installing the Python dependencies. They are diagnostic
examples, not checks that the current implementation is expected to pass.

### Combined rotation and translation

The displacement in `shift_pose_dgm` is already expressed in the new body frame,
but the OpenCV transform currently composes it as `R @ T`. This rotates the
translation a second time when the robot both turns and translates.

```python
import math
import numpy as np
from libs.dgm import shift_pose_dgm

image = np.zeros((101, 101), dtype=np.float32)
image[50, 50] = 1.0
shifted = shift_pose_dgm(
    image, (0.0, 0.0, 0.0), (1.0, 0.0, math.pi / 2),
    fill_val=0.0, interpolation="nearest",
)
print(np.argwhere(shifted == 1.0).tolist())
```

With 0.1 m cells, the observed result is `[[50, 60]]`. The point is at the old
robot origin; after moving 1 m forward and turning 90 degrees, it is 1 m to the
new robot's left. The expected image coordinate is therefore `[[40, 50]]`.
Review `T @ R` composition and confirm the convention against real pose/cloud
pairs. Any change should also reconcile the NumPy fallback and the legacy OGM
transform, then test translation, rotation, and combined motion together.

### Dynamic threshold at the measurement-confidence ceiling

Both `FREE_CONF` and `OCC_CONF` are 0.7, and `DYNAMIC_THRESHOLD` is also 0.7.
With these measurements, conflict cannot exceed 0.7 for normalized prior masses.
A partially confident prior therefore fails the dynamic threshold even when a
new measurement contradicts it.

```python
import numpy as np
from libs.dgm import DYNAMIC_THRESHOLD, update_dgm

prior = np.array([[[1.0, 0.0, 0.0]]], dtype=np.float32)
free = np.array([[[0.3, 0.0, 0.7]]], dtype=np.float32)
occupied = np.array([[[0.3, 0.7, 0.0]]], dtype=np.float32)
for count in range(1, 15):
    prior, _ = update_dgm(prior, free)
    _, conflict = update_dgm(prior, occupied)
    print(count, float(conflict[0, 0]),
          bool(conflict[0, 0] >= DYNAMIC_THRESHOLD))
```

For this example, 13 accumulated free observations give a conflict of about
0.69999993 and no dynamic label. At observation 14, the prior rounds to complete
free-space confidence in float32 and the dynamic comparison becomes true. This
illustrates dependence on prior saturation; it is not a measured sensor latency.
Table V of the paper reports a 0.65 baseline on its real-world dataset, whereas
the public code uses 0.7. Reconcile the confidence/threshold pair with the
experimental configuration and validate it on annotated sequences before
changing the default.

## Reproduction work still needed

1. Confirm the intended coordinate conventions, sensor visibility model, and
   parameter sets; resolve the two edge cases above with regression checks.
2. Provide redistributable example data, exact topic/frame definitions, and
   configuration files for each experimental setup.
3. Publish the evaluation scripts and annotations behind the paper's tables,
   including hardware/software versions and timing boundaries.
4. Reconcile the released map memory implementation with the paper's description
   before claiming the same runtime or memory behavior for this Python version.

Run `python scripts/smoke_test.py` to check imports, ordinary mass fusion, RGB
rendering, and a serial pipeline with temporary output. This does not validate
ROS synchronization, map alignment under motion, dynamic detection accuracy,
flight integration, or the paper's benchmark results.
