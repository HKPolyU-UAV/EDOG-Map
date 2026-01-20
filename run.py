from __future__ import annotations

import os
import time
from collections import deque
from typing import Tuple

import numpy as np
import tqdm
from pystream import Pipeline

from pipeline.data import AppData
from pipeline.sensor_reader import SensorReaderStage
from pipeline.road_segmentation import RoadSegmentationStage
from pipeline.dgm_map import DGMMapStage
from pipeline.output_writer import OutputWriterStage


MODEL_PATH = "../pretrained/deeplab_model.pb"
MODEL_INPUT_SIZE = 513

ROAD_HEIGHT_THRESHOLD = 0.15 
OUTPUT_DIR = "results"

CROP_RH = 3
CROP_RW = 4

def _pointcloud2_to_xyz_i(point_cloud2_mod, msg) -> np.ndarray:
    names = {f.name for f in msg.fields}
    has_intensity = "intensity" in names

    if has_intensity:
        arr = np.array(
            list(point_cloud2_mod.read_points(
                msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
            )),
            dtype=np.float32
        )
        if arr.size == 0:
            return np.zeros((0, 4), dtype=np.float32)
        return arr 
    else:
        xyz = np.array(
            list(point_cloud2_mod.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )),
            dtype=np.float32
        )
        if xyz.size == 0:
            return np.zeros((0, 4), dtype=np.float32)
        out = np.zeros((xyz.shape[0], 4), dtype=np.float32)
        out[:, :3] = xyz
        return out



def _odom_to_xyyaw(euler_from_quaternion, odom_msg) -> Tuple[float, float, float]:
    p = odom_msg.pose.pose.position
    q = odom_msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
    return float(p.x), float(p.y), float(yaw)


def main():
    try:
        import rospy 
        import message_filters  
        from sensor_msgs.msg import PointCloud2  
        from nav_msgs.msg import Odometry  
        from sensor_msgs import point_cloud2  
        from tf.transformations import euler_from_quaternion 
    except Exception as e:
        raise RuntimeError(
            "找不到 ROS Python 依赖。请在当前终端先执行：\n"
            "  source /opt/ros/noetic/setup.bash\n"
            "  source ~/catkin_ws/devel/setup.bash\n"
            "再运行：python3 run.py\n"
        ) from e

    rospy.init_node("uav_lidar_dgm_run", anonymous=False)

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # ---------------- Pipeline ----------------
    sensor_reader_stage = SensorReaderStage(None, CROP_RW, CROP_RH)
    road_segmentation_stage = RoadSegmentationStage(MODEL_PATH, MODEL_INPUT_SIZE) 
    dgm_map_stage = DGMMapStage(ROAD_HEIGHT_THRESHOLD)  
    output_writer_stage = OutputWriterStage(OUTPUT_DIR)

    app_pipeline = Pipeline()
    app_pipeline.add(sensor_reader_stage)
    app_pipeline.add(road_segmentation_stage)
    app_pipeline.add(dgm_map_stage)
    app_pipeline.add(output_writer_stage)

    app_pipeline.parallelize()
    # app_pipeline.serialize()

    lidar_topic = rospy.get_param("~lidar_topic", "/cloud_registered_body")
    odom_topic = rospy.get_param("~odom_topic", "/Odometry")
    max_frames = int(rospy.get_param("~max_frames", 10**9))

    sync_queue = int(rospy.get_param("~sync_queue", 30))
    sync_slop = float(rospy.get_param("~sync_slop", 0.05))

    max_points = int(rospy.get_param("~max_points", 30000))
    frame_queue = int(rospy.get_param("~frame_queue", 200))

    z_min = float(rospy.get_param("~z_min", 0))
    z_max = float(rospy.get_param("~z_max",  5))

    use_forward_right = bool(rospy.get_param("~use_forward_right", False))

    def filter_z(pts: np.ndarray) -> np.ndarray:
        if pts.shape[0] == 0:
            return pts
        z = pts[:, 2]
        m = (z >= z_min) & (z <= z_max)
        return pts[m]

    def convert_pose(pose_xyyaw: Tuple[float, float, float]) -> Tuple[float, float, float]:
        if not use_forward_right:
            return pose_xyyaw
        x, y, yaw = pose_xyyaw
        return (x, -y, -yaw)

    def maybe_subsample(pts: np.ndarray) -> np.ndarray:
        if pts.shape[0] <= max_points:
            return pts
        idx = np.linspace(0, pts.shape[0] - 1, max_points).astype(np.int64)
        return pts[idx]

    q = deque(maxlen=frame_queue)

    sub_lidar = message_filters.Subscriber(lidar_topic, PointCloud2)
    sub_odom = message_filters.Subscriber(odom_topic, Odometry)
    sync = message_filters.ApproximateTimeSynchronizer(
        [sub_lidar, sub_odom], queue_size=sync_queue, slop=sync_slop
    )

    def cb(lidar_msg: PointCloud2, odom_msg: Odometry):
        xyz_i = _pointcloud2_to_xyz_i(point_cloud2, lidar_msg)
        xyz_i = filter_z(xyz_i)
        xyz_i = maybe_subsample(xyz_i)
        pose = convert_pose(_odom_to_xyyaw(euler_from_quaternion, odom_msg))
        q.append((xyz_i, pose))

    sync.registerCallback(cb)

    prev_pose = (0.0, 0.0, 0.0)
    old_idx = 0

    try:
        for idx in tqdm.tqdm(range(max_frames)):
            if rospy.is_shutdown():
                break

            while (not rospy.is_shutdown()) and (len(q) == 0):
                rospy.sleep(0.001)
            if rospy.is_shutdown():
                break

            lidar_raw, pose = q.popleft()

            app_data = AppData()
            app_data.metadata.data_idx = idx
            app_data.metadata.previous_data_idx = old_idx

            app_data.input_data.input_image = np.array([])
            app_data.input_data.lidar_in_cam = np.array([])
            app_data.input_data.lidar_2d = np.array([])

            app_data.input_data.lidar_raw = lidar_raw
            app_data.input_data.previous_pose = prev_pose
            app_data.input_data.pose = pose
            prev_pose = pose

            app_pipeline.forward(app_data)
            old_idx = idx

    finally:
        time.sleep(0.5)
        app_pipeline.cleanup()


if __name__ == "__main__":
    main()
