# Usage guide

[Back to README](../README.md)

This guide describes the current Python implementation. `run.py` subscribes to ROS 1 point clouds and odometry, builds a robot-centric **2D** evidential grid, and writes color-coded maps to disk. It does not publish a ROS map, plan trajectories, or control a UAV.

## 1. Environment

The entry point uses ROS 1 APIs (`rospy`, `message_filters`, `sensor_msgs`, `nav_msgs`, and `tf`). ROS Noetic on Ubuntu 20.04 with its system Python 3.8 is the reference setup for these instructions; this is a compatibility target, not a claim of a newly verified full ROS installation. Noetic reached [end of life on May 31, 2025](https://discourse.ros.org/t/new-packages-for-noetic-2025-05-29-final/44003). Use an existing ROS 1 environment for this release; there is no ROS 2 entry point.

For the non-ROS Python core, use Python 3.8-3.11. The `pystream-pipeline==0.2.0` dependency requires pandas older than 2; Python 3.12+ is not covered by these installation instructions.

In an existing Ubuntu 20.04 / ROS Noetic installation, ensure the following packages are available:

```bash
sudo apt-get update
sudo apt-get install python3-pip python3-venv \
  ros-noetic-rospy ros-noetic-message-filters \
  ros-noetic-sensor-msgs ros-noetic-nav-msgs ros-noetic-tf
```

Clone the repository and create an environment that can see the system ROS Python packages:

```bash
git clone https://github.com/HKPolyU-UAV/EDOG-Map.git
cd EDOG-Map
source /opt/ros/noetic/setup.bash
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

If your sensor driver or odometry package is built in a catkin workspace, also source that workspace's `devel/setup.bash` in each runtime terminal.

The distribution named **`pystream-pipeline`** supplies the `pystream` import; installing the unrelated package named `pystream` is not a substitute. OpenCV is installed without GUI components because this entry point writes files. TensorFlow and pretrained segmentation weights are unnecessary for the LiDAR-only entry point: it supplies an empty image and bypasses image inference. The legacy image/road-segmentation helpers require additional setup and are not a complete supported camera pipeline.

Check the non-ROS components from the repository root:

```bash
python -m compileall -q run.py libs pipeline scripts
python scripts/smoke_test.py
```

These checks exercise Python syntax and a small synthetic core/pipeline case. They do not validate ROS synchronization, sensor calibration, flight behavior, or the paper's benchmark results.

## 2. Input contract

| Input | Default topic | Required content |
| --- | --- | --- |
| LiDAR | `/cloud_registered_body` | `sensor_msgs/PointCloud2` with `x`, `y`, `z` fields in meters. `intensity` is optional and is not used by the mapper. |
| Odometry | `/Odometry` | `nav_msgs/Odometry`; the node reads position `x`, `y` and quaternion-derived yaw. |

Both messages need compatible `header.stamp` values. They are paired by `ApproximateTimeSynchronizer` with a default tolerance of 0.05 seconds.

Supply point clouds in the local/body coordinate system used by the odometry pose: x forward and y left for the default convention, with z consistent with the configured height filter. The node does **not** inspect `frame_id`, look up TF transforms, or apply LiDAR-to-body extrinsics. A cloud already expressed in a fixed world frame must be transformed upstream before use. Align the sensor origin and axes with the pose convention upstream as well.

Only planar translation and yaw compensate motion; z position, roll, and pitch are not used. The default height filter retains points with `0 <= z <= 5` in the **incoming point-cloud frame**, then projects retained points onto x-y. Adjust this interval to your sensor mounting and scene; it is not an automatic ground-removal step.

`_use_forward_right:=true` changes the pose to `(x, -y, -yaw)` only. It does not reflect the point cloud. Leave it false unless the input data have already been made consistent with this convention.

## 3. Run with live topics

Start a ROS master in a separate terminal, unless your system already has one:

```bash
source /opt/ros/noetic/setup.bash
roscore
```

Start your LiDAR and odometry source, then check that the expected topics are being published:

```bash
rostopic type /cloud_registered_body
rostopic type /Odometry
rostopic hz /cloud_registered_body
rostopic hz /Odometry
```

Run the mapper from the repository root in a terminal with ROS sourced and `.venv` activated:

```bash
python run.py \
  _lidar_topic:=/cloud_registered_body \
  _odom_topic:=/Odometry \
  _z_min:=0.0 \
  _z_max:=5.0 \
  _max_frames:=1000
```

This runs until 1,000 synchronized frames have been submitted or the node is stopped with Ctrl+C. Without new synchronized messages it waits; it does not automatically stop when a data source ends. The fixed node name is `uav_lidar_dgm_run`. Output goes to `results/` relative to the working directory.

## 4. Replay an existing ROS bag

The repository does not ship a rosbag or a dataset downloader. Use a recording that satisfies the input contract above. Inspect its topic names before replay:

```bash
rosbag info /path/to/recording.bag
```

With `roscore` running, set simulated time **before starting the mapper**, launch the mapper as above with the bag's topic names, and start playback in another terminal:

```bash
rosparam set /use_sim_time true
# Start the mapper in another terminal, then:
rosbag play --clock /path/to/recording.bag
```

Stop the mapper after playback ends if `_max_frames` has not been reached. Before a later live-sensor run, stop the mapper, set `rosparam set /use_sim_time false`, and restart it.

## 5. Runtime parameters

Use ROS private-parameter syntax, for example `_sync_slop:=0.1`. These parameters are read by `run.py`:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `lidar_topic` | `/cloud_registered_body` | Point-cloud topic. |
| `odom_topic` | `/Odometry` | Odometry topic. |
| `max_frames` | `1000000000` | Maximum synchronized frames submitted to the pipeline. |
| `sync_queue` | `30` | Synchronizer queue length. |
| `sync_slop` | `0.05` | Allowed timestamp difference in seconds. |
| `max_points` | `30000` | Maximum points after height filtering; larger clouds are uniformly subsampled by array index. |
| `frame_queue` | `200` | Pending synchronized frames; appending to a full queue drops the oldest frame. |
| `z_min` | `0` | Inclusive lower height bound in meters in the incoming cloud frame. |
| `z_max` | `5` | Inclusive upper height bound. |
| `use_forward_right` | `false` | Pose-only sign conversion described above. |

Use positive queue lengths and point limits, a positive frame limit, and `z_min <= z_max`. The current entry point does not validate these combinations.

Map geometry and evidence settings are constants in [`libs/dgm.py`](../libs/dgm.py), not ROS parameters:

| Constant | Default | Meaning |
| --- | --- | --- |
| `MAP_WIDTH` | `100` m | Square map width. |
| `RESOLUTION` | `0.1` m | Cartesian grid resolution; defaults produce a 1000 x 1000 grid. |
| `MAX_RANGE` | `50` m | Cells at or beyond this radial distance remain unknown in a new measurement. |
| `ALPHA` | `1` m | Polar range-bin size. |
| `BHETA` | `1` degree | Polar angular-bin size, represented in radians in code. |
| `FREE_CONF` | `0.7` | Free-space measurement mass. |
| `OCC_CONF` | `0.7` | Occupied-space measurement mass. |
| `DYNAMIC_THRESHOLD` | `0.7` | Conflict threshold used for red pixels in the prediction. |

Edit constants before starting Python so the derived arrays are rebuilt consistently. `OUTPUT_DIR = "results"` is defined in `run.py`. The existing `ROAD_HEIGHT_THRESHOLD`, crop settings, and segmentation model settings do not control the active LiDAR DGM update.

## 6. Outputs

Each processed frame is written as:

```text
results/
  000000_dgm_pred.npy
  000000_dgm_pred.png
  000001_dgm_pred.npy
  000001_dgm_pred.png
  ...
```

The `.npy` files contain a **uint8 RGB visualization**, shape `(1000, 1000, 3)` with default geometry. They do not contain the underlying evidence masses. PNG output is enabled when Matplotlib is available; a PNG save failure is currently suppressed by the writer.

| RGB | Meaning |
| --- | --- |
| `(123, 123, 123)` | Unknown has the largest mass. |
| `(0, 0, 0)` | Occupied has the largest mass. |
| `(255, 255, 255)` | Free has the largest mass. |
| `(255, 0, 0)` | Conflict mass meets `DYNAMIC_THRESHOLD`; overrides the above colors. |

For programmatic integration, `AppData.output_data.dgm` holds float32 channels in the order **unknown, occupied, free**, and `dynamic_mass` holds the 2D conflict array. Add an explicit writer or publisher if those arrays are needed downstream. The current disk writer saves only `dgm_pred`.

Read a saved visualization with:

```python
import numpy as np

rgb = np.load("results/000000_dgm_pred.npy", allow_pickle=False)
print(rgb.shape, rgb.dtype)
```

Frame numbering restarts at zero on every run, so running again in the same directory overwrites matching files. Archive results before the next run. The reported `Average throughput` includes intervals between output writes, including upstream waits and disk I/O; it is not the paper's isolated mapping latency.

## 7. Troubleshooting and implementation scope

| Symptom | Checks |
| --- | --- |
| Missing ROS Python modules | Source ROS and the relevant catkin workspace; use the ROS-compatible interpreter and an environment created with `--system-site-packages`. |
| `Pipeline` or `Stage` import fails | Install `pystream-pipeline==0.2.0` into the interpreter used to run the node. |
| pandas installation fails on a newer Python | Use Python 3.8-3.11 for this dependency set. |
| No output files | Check topic names, message types, timestamps, synchronizer tolerance, and whether both inputs are publishing. |
| Empty or distorted maps | Check the incoming frame, extrinsics, height limits, coordinate signs, and odometry continuity. |
| Missing PNG but `.npy` exists | Verify Matplotlib imports and disk permissions; inspect the array directly. |
| Old frames or inconsistent output count | The input deque can drop frames when full. The threaded pipeline and fixed 0.5-second shutdown delay do not guarantee that every submitted frame is flushed to disk. |

The current sensor model treats angular bins without retained returns as free when a nonempty scan is processed; an entirely empty input is returned as unknown. Missing returns can arise from limited field of view, filtering, or subsampling, so this assumption needs review for each sensor. See [implementation notes](implementation-notes.md) for known algorithm edge cases and their reproducible checks.

The released code provides a local 2D map with planar pose compensation. It does not include the complete experimental data/configurations, a general 3D flight mapping interface, velocity tracking, a ROS map publisher, or a navigation/control stack. Use the paper for the full method and evaluation, and distinguish those reported results from measurements made with this code.
