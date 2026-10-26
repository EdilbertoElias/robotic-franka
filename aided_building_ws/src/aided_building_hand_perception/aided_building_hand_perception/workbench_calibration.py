"""Estimate the camera pose relative to an ArUco marker lying on the workbench.

The marker frame becomes the `workbench` frame (origin at the marker center,
z pointing up out of the table). The result is written to a YAML file that
hand_perception.launch.py turns into the static TF workbench -> camera.
"""
import datetime
import math
import os

import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from aided_building_hand_perception.aruco import (
    detect_markers, estimate_marker_pose, get_dictionary)
from aided_building_hand_perception.geometry import (
    average_quaternions, invert_transform, matrix_from_quaternion,
    quaternion_from_matrix)

DEFAULT_OUTPUT = '~/.ros/aided_building/workbench_camera.yaml'


class WorkbenchCalibration(Node):

    def __init__(self):
        super().__init__('workbench_calibration')
        self.dictionary_name = self.declare_parameter('marker_dictionary', 'DICT_4X4_50').value
        self.marker_id = int(self.declare_parameter('marker_id', 0).value)
        self.marker_size = float(self.declare_parameter('marker_size', 0.10).value)
        self.samples = int(self.declare_parameter('samples', 60).value)
        self.max_error = float(self.declare_parameter('max_reprojection_error', 2.0).value)
        self.plane_frame = self.declare_parameter('plane_frame', 'workbench').value
        self.output = os.path.expanduser(self.declare_parameter('output', DEFAULT_OUTPUT).value)

        self.dictionary = get_dictionary(self.dictionary_name)
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.translations = []
        self.quaternions = []
        self.errors = []
        self.camera_frame = None
        self.done = False

        image_qos = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(CameraInfo, '/camera_info', self.on_camera_info, 1)
        self.create_subscription(Image, '/image_raw', self.on_image, image_qos)
        self.get_logger().info(
            f'Looking for {self.dictionary_name} id={self.marker_id} '
            f'({self.marker_size * 1000:.0f} mm); collecting {self.samples} samples')

    def on_camera_info(self, msg):
        if not any(msg.k):
            self.get_logger().error(
                'camera_info has no intrinsics; calibrate the camera first',
                throttle_duration_sec=5.0)
            return
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def on_image(self, msg):
        if self.done or self.camera_matrix is None:
            return
        gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        corners = detect_markers(gray, self.dictionary).get(self.marker_id)
        if corners is None:
            self.get_logger().warn('Marker not visible', throttle_duration_sec=2.0)
            return
        pose = estimate_marker_pose(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
        if pose is None or pose[1] > self.max_error:
            return

        plane_from_camera = invert_transform(pose[0])
        self.camera_frame = msg.header.frame_id
        self.translations.append(plane_from_camera[:3, 3])
        self.quaternions.append(quaternion_from_matrix(plane_from_camera[:3, :3]))
        self.errors.append(pose[1])
        if len(self.errors) % 10 == 0:
            self.get_logger().info(f'{len(self.errors)}/{self.samples} samples')
        if len(self.errors) >= self.samples:
            self.finish()

    def finish(self):
        self.done = True
        translations = np.array(self.translations)
        translation = np.median(translations, axis=0)
        quaternion = average_quaternions(self.quaternions)
        spread_mm = np.std(translations, axis=0) * 1000.0

        rotation = matrix_from_quaternion(quaternion)
        optical_axis = rotation[:, 2]
        tilt = math.degrees(math.acos(np.clip(-optical_axis[2], -1.0, 1.0)))

        result = {
            'workbench_to_camera': {
                'parent_frame': self.plane_frame,
                'child_frame': self.camera_frame,
                'translation': [round(float(v), 5) for v in translation],
                'rotation_xyzw': [round(float(v), 6) for v in quaternion],
            },
            'calibration': {
                'date': datetime.datetime.now().isoformat(timespec='seconds'),
                'marker_dictionary': self.dictionary_name,
                'marker_id': self.marker_id,
                'marker_size_m': self.marker_size,
                'samples': len(self.errors),
                'mean_reprojection_error_px': round(float(np.mean(self.errors)), 3),
                'translation_std_mm': [round(float(v), 2) for v in spread_mm],
            },
        }
        os.makedirs(os.path.dirname(self.output), exist_ok=True)
        with open(self.output, 'w') as output:
            yaml.safe_dump(result, output, sort_keys=False)

        self.get_logger().info(
            f'Camera at ({translation[0]:+.3f}, {translation[1]:+.3f}, {translation[2]:+.3f}) m '
            f'in {self.plane_frame}; height {translation[2]:.3f} m, optical axis '
            f'{tilt:.0f} deg from vertical; jitter {np.max(spread_mm):.1f} mm; '
            f'reprojection {np.mean(self.errors):.2f} px')
        self.get_logger().info(f'Saved {self.output}')


def main(args=None):
    rclpy.init(args=args)
    node = WorkbenchCalibration()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
