"""Detect hands with MediaPipe and express them in the robot base frame.

A monocular camera has no depth, so every landmark is back-projected as a ray
and intersected with a plane parallel to the workbench (z = plane_height in
plane_frame). This holds while the hands work on or just above the table.

Each hand is published as a zone: the smallest oriented rectangle around its
landmarks seen from above, grown by zone_margin and extruded from the table up
to zone_height. Distances to the robot are measured to that zone.
"""
import math
import os
import time

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import (
    Point, Point32, PointStamped, PolygonStamped, Pose, PoseArray, Quaternion, Vector3,
    Vector3Stamped)
from mediapipe.tasks.python import BaseOptions, vision
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, Float32
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

from aided_building_hand_perception.geometry import (
    OneEuroFilter, distance_to_zone, hand_zone, intersect_rays_with_plane, invert_transform,
    matrix_from_rpy, pixel_rays, quaternion_from_matrix, transform_from_msg)

PALM_IDS = [0, 5, 9, 13, 17]
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15),
    (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17),
]
BOX_EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]
SIDES = ('left', 'right')
SIDE_COLORS = {'left': (0.1, 0.6, 1.0), 'right': (1.0, 0.5, 0.1)}
DEFAULT_ROBOT_LINKS = [f'fr3_link{i}' for i in range(0, 9)]
FR3_REACH = 0.855


def to_point(p):
    return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))


def bgr(side):
    r, g, b = SIDE_COLORS[side]
    return (int(b * 255), int(g * 255), int(r * 255))


def put_text(image, text, origin, color, scale=0.5):
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3)
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1)


class HandDetector(Node):

    def __init__(self):
        super().__init__('hand_detector')
        model_path = os.path.expanduser(self.declare_parameter(
            'model_path', '~/models/hand_landmarker.task').value)
        self.base_frame = self.declare_parameter('base_frame', 'fr3_link0').value
        self.plane_frame = self.declare_parameter('plane_frame', 'workbench').value
        self.plane_height = float(self.declare_parameter('plane_height', 0.02).value)
        self.zone_margin = float(self.declare_parameter('zone_margin', 0.03).value)
        self.zone_height = float(self.declare_parameter('zone_height', 0.10).value)
        self.tool_frame = self.declare_parameter('tool_frame', 'fr3_link8').value
        self.image_is_mirrored = bool(self.declare_parameter('image_is_mirrored', False).value)
        self.robot_links = list(self.declare_parameter('robot_links', DEFAULT_ROBOT_LINKS).value)
        self.lost_timeout = float(self.declare_parameter('lost_timeout', 0.5).value)
        self.top_view_scale = float(self.declare_parameter('top_view_scale', 400.0).value)
        self.top_view_size = [int(v) for v in self.declare_parameter(
            'top_view_size', [560, 560]).value]
        min_cutoff = float(self.declare_parameter('filter_min_cutoff', 1.0).value)
        beta = float(self.declare_parameter('filter_beta', 2.0).value)
        image_topic = self.declare_parameter('image_topic', '/image_raw').value
        info_topic = self.declare_parameter('camera_info_topic', '/camera_info').value

        self.landmarker = vision.HandLandmarker.create_from_options(
            vision.HandLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=model_path),
                running_mode=vision.RunningMode.VIDEO,
                num_hands=int(self.declare_parameter('num_hands', 2).value),
                min_hand_detection_confidence=float(
                    self.declare_parameter('min_detection_confidence', 0.5).value),
                min_tracking_confidence=float(
                    self.declare_parameter('min_tracking_confidence', 0.5).value)))

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.last_mp_ms = -1
        self.filters = {side: OneEuroFilter(min_cutoff, beta) for side in SIDES}
        self.last_seen = {side: None for side in SIDES}
        self.frame_times = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Large images are fragmented by DDS; BEST_EFFORT drops whole frames
        image_qos = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(CameraInfo, info_topic, self.on_camera_info, 1)
        self.create_subscription(Image, image_topic, self.on_image, image_qos)

        self.poses_pub = self.create_publisher(PoseArray, '/hands/poses', 10)
        self.zones_pub = self.create_publisher(Detection3DArray, '/hands/zones', 10)
        self.palm_pubs = {
            side: self.create_publisher(PointStamped, f'/hands/{side}/palm', 10)
            for side in SIDES}
        self.velocity_pubs = {
            side: self.create_publisher(Vector3Stamped, f'/hands/{side}/velocity', 10)
            for side in SIDES}
        self.footprint_pubs = {
            side: self.create_publisher(PolygonStamped, f'/hands/{side}/zone', 10)
            for side in SIDES}
        self.distance_pub = self.create_publisher(Float32, '/hands/min_distance', 10)
        self.tool_distance_pub = self.create_publisher(Float32, '/hands/tool_distance', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/hands/markers', 10)
        self.debug_pub = self.create_publisher(Image, '/hands/debug_image', image_qos)
        self.top_view_pub = self.create_publisher(Image, '/hands/top_view', image_qos)

        self.get_logger().info(
            f'Hands projected on z={self.plane_height:.3f} m of {self.plane_frame}, '
            f'zones {self.zone_margin * 100:.0f} cm margin x {self.zone_height * 100:.0f} cm '
            f'high, published in {self.base_frame}')

    def on_camera_info(self, msg):
        if not any(msg.k):
            self.get_logger().error(
                'camera_info has no intrinsics; calibrate the camera first',
                throttle_duration_sec=5.0)
            return
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def lookup(self, target, source):
        try:
            return transform_from_msg(
                self.tf_buffer.lookup_transform(target, source, Time()).transform)
        except TransformException:
            return None

    def on_image(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn('Waiting for camera_info', throttle_duration_sec=5.0)
            return

        now_s = time.monotonic()
        self.frame_times = [t for t in self.frame_times if now_s - t < 1.0] + [now_s]
        # With simulated time the image (wall clock) and TF (Gazebo clock) disagree
        if self.get_parameter('use_sim_time').value:
            stamp = self.get_clock().now().to_msg()
        else:
            stamp = msg.header.stamp

        rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        height, width = rgb.shape[:2]
        mp_ms = max(Time.from_msg(msg.header.stamp).nanoseconds // 1_000_000, self.last_mp_ms + 1)
        self.last_mp_ms = mp_ms
        result = self.landmarker.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), mp_ms)

        plane_from_camera = self.lookup(self.plane_frame, msg.header.frame_id)
        base_from_plane = self.lookup(self.base_frame, self.plane_frame)
        if plane_from_camera is None:
            self.get_logger().warn(
                f'No TF {self.plane_frame} -> {msg.header.frame_id}; is the workbench '
                f'calibration loaded?', throttle_duration_sec=5.0)
        if base_from_plane is None:
            self.get_logger().warn(
                f'No TF {self.base_frame} -> {self.plane_frame}; publishing in '
                f'{self.plane_frame}', throttle_duration_sec=5.0)
            base_from_plane = np.eye(4)
            output_frame = self.plane_frame
        else:
            output_frame = self.base_frame

        scene = {
            'plane_from_camera': plane_from_camera,
            'base_from_plane': base_from_plane,
            'links': self.positions_in_plane(self.robot_links),
            'tool': next(iter(self.positions_in_plane([self.tool_frame])), None),
        }
        hands = []
        for landmarks, handedness in zip(result.hand_landmarks, result.handedness):
            side = handedness[0].category_name.lower()
            if not self.image_is_mirrored:
                # MediaPipe labels assume a mirrored (selfie) image
                side = 'right' if side == 'left' else 'left'
            pixels = np.array([(p.x * width, p.y * height) for p in landmarks])
            hand = {'side': side, 'score': handedness[0].score, 'pixels': pixels}
            if plane_from_camera is not None:
                hand.update(self.locate(pixels, scene, side, now_s))
            hands.append(hand)

        for side in SIDES:
            seen = self.last_seen[side]
            if seen is not None and now_s - seen > self.lost_timeout:
                self.filters[side].reset()
                self.last_seen[side] = None

        self.publish(hands, stamp, output_frame)
        if self.debug_pub.get_subscription_count() > 0:
            self.publish_debug(msg, rgb, hands, scene, output_frame)
        if self.top_view_pub.get_subscription_count() > 0 and plane_from_camera is not None:
            self.publish_top_view(msg, hands, scene)

    def positions_in_plane(self, frames):
        positions = []
        for frame in frames:
            transform = self.lookup(self.plane_frame, frame)
            if transform is not None:
                positions.append(transform[:3, 3])
        return positions

    def locate(self, pixels, scene, side, now_s):
        palm_pixel = pixels[PALM_IDS].mean(axis=0)
        rays = pixel_rays(np.vstack([palm_pixel, pixels]), self.camera_matrix, self.dist_coeffs)
        points = intersect_rays_with_plane(scene['plane_from_camera'], rays, self.plane_height)
        landmarks = points[1:]
        visible = ~np.isnan(landmarks).any(axis=1)
        if np.isnan(points[0]).any() or visible.sum() < 3:
            return {}

        center, size, yaw, corners = hand_zone(landmarks[visible, :2], self.zone_margin)
        box_plane = np.vstack([
            np.column_stack([corners, np.zeros(4)]),
            np.column_stack([corners, np.full(4, self.zone_height)])])

        base_from_plane = scene['base_from_plane']
        rotation, translation = base_from_plane[:3, :3], base_from_plane[:3, 3]
        palm, velocity = self.filters[side](now_s, rotation @ points[0] + translation)
        self.last_seen[side] = now_s

        box_pose = Pose(
            position=to_point(rotation @ [center[0], center[1], self.zone_height / 2] + translation),
            orientation=Quaternion(**dict(zip(
                'xyzw', quaternion_from_matrix(rotation @ matrix_from_rpy(0.0, 0.0, yaw))))))
        located = {
            'palm': palm,
            'velocity': velocity,
            'landmarks': landmarks @ rotation.T + translation,
            'landmarks_plane': landmarks,
            'box': box_plane @ rotation.T + translation,
            'box_plane': box_plane,
            'box_pose': box_pose,
            'box_size': Vector3(x=float(size[0]), y=float(size[1]), z=self.zone_height),
        }
        if scene['links']:
            located['distance'] = min(
                distance_to_zone(p, corners, 0.0, self.zone_height) for p in scene['links'])
        if scene['tool'] is not None:
            located['tool_distance'] = distance_to_zone(
                scene['tool'], corners, 0.0, self.zone_height)
        return located

    def publish(self, hands, stamp, frame):
        poses = PoseArray()
        poses.header.stamp = stamp
        poses.header.frame_id = frame
        zones = Detection3DArray(header=poses.header)
        located = [hand for hand in hands if 'palm' in hand]
        for hand in located:
            side = hand['side']
            pose = Pose(position=to_point(hand['palm']))
            pose.orientation.w = 1.0
            poses.poses.append(pose)

            point = PointStamped(header=poses.header, point=to_point(hand['palm']))
            self.palm_pubs[side].publish(point)

            velocity = Vector3Stamped(header=poses.header)
            velocity.vector = Vector3(**dict(zip('xyz', (float(v) for v in hand['velocity']))))
            self.velocity_pubs[side].publish(velocity)

            footprint = PolygonStamped(header=poses.header)
            footprint.polygon.points = [
                Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in hand['box'][:4]]
            self.footprint_pubs[side].publish(footprint)

            detection = Detection3D(header=poses.header, id=side)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = side
            hypothesis.hypothesis.score = float(hand['score'])
            hypothesis.pose.pose = hand['box_pose']
            detection.results.append(hypothesis)
            detection.bbox.center = hand['box_pose']
            detection.bbox.size = hand['box_size']
            zones.detections.append(detection)

        self.poses_pub.publish(poses)
        self.zones_pub.publish(zones)
        for key, publisher in (('distance', self.distance_pub),
                               ('tool_distance', self.tool_distance_pub)):
            values = [hand[key] for hand in located if key in hand]
            if values:
                publisher.publish(Float32(data=float(min(values))))
        self.markers_pub.publish(self.make_markers(located, poses.header))

    def make_markers(self, hands, header):
        markers = MarkerArray()
        markers.markers.append(Marker(header=header, action=Marker.DELETEALL))
        lifetime = Duration(seconds=0.3).to_msg()
        for index, hand in enumerate(hands):
            r, g, b = SIDE_COLORS[hand['side']]
            color = ColorRGBA(r=r, g=g, b=b, a=0.9)
            points = [to_point(p) for p in hand['landmarks']]
            corners = [to_point(p) for p in hand['box']]

            visible = np.isfinite(hand['landmarks']).all(axis=1)
            skeleton = Marker(header=header, ns='skeleton', id=index, type=Marker.LINE_LIST)
            skeleton.scale.x = 0.005
            skeleton.color = color
            skeleton.points = [points[i] for pair in HAND_CONNECTIONS
                               if visible[list(pair)].all() for i in pair]

            box = Marker(header=header, ns='zone', id=index, type=Marker.LINE_LIST)
            box.scale.x = 0.004
            box.color = color
            box.points = [corners[i] for pair in BOX_EDGES for i in pair]

            volume = Marker(header=header, ns='zone_volume', id=index, type=Marker.CUBE)
            volume.pose = hand['box_pose']
            volume.scale = hand['box_size']
            volume.color = ColorRGBA(r=r, g=g, b=b, a=0.25)

            label = Marker(header=header, ns='label', id=index, type=Marker.TEXT_VIEW_FACING)
            label.pose.position = to_point(hand['box'][4:].mean(axis=0) + [0.0, 0.0, 0.04])
            label.scale.z = 0.03
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = hand['side'] + (
                f" tool {hand['tool_distance']:.2f} m" if 'tool_distance' in hand else '')

            for marker in (skeleton, box, volume, label):
                if marker is not volume:
                    marker.pose.orientation.w = 1.0
                marker.lifetime = lifetime
                markers.markers.append(marker)
        return markers

    def publish_debug(self, msg, rgb, hands, scene, frame):
        image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        camera_from_plane = (
            invert_transform(scene['plane_from_camera'])
            if scene['plane_from_camera'] is not None else None)
        for hand in hands:
            color = bgr(hand['side'])
            if 'box_plane' in hand and camera_from_plane is not None:
                rvec = cv2.Rodrigues(camera_from_plane[:3, :3])[0]
                projected, _ = cv2.projectPoints(
                    hand['box_plane'], rvec, camera_from_plane[:3, 3],
                    self.camera_matrix, self.dist_coeffs)
                corners = projected.reshape(-1, 2).astype(int)
                for a, b in BOX_EDGES:
                    cv2.line(image, tuple(corners[a]), tuple(corners[b]), color, 1, cv2.LINE_AA)
            points = [tuple(int(v) for v in p) for p in hand['pixels']]
            for a, b in HAND_CONNECTIONS:
                cv2.line(image, points[a], points[b], color, 2)
            for point in points:
                cv2.circle(image, point, 3, (255, 255, 255), -1)
            text = f"{hand['side']} {hand['score']:.2f}"
            if 'palm' in hand:
                x, y, z = hand['palm']
                text += f' ({x:+.2f}, {y:+.2f}, {z:+.2f}) m'
            if 'tool_distance' in hand:
                text += f" tool {hand['tool_distance']:.2f} m"
            put_text(image, text, (points[0][0] - 60, points[0][1] + 25), color)
        put_text(image, f'{len(self.frame_times)} fps  frame: {frame}', (8, 22),
                 (255, 255, 255), 0.55)
        debug = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        debug.header = msg.header
        self.debug_pub.publish(debug)

    def publish_top_view(self, msg, hands, scene):
        """Bird's-eye view of the workbench plane: +x up, +y left."""
        width, height = self.top_view_size
        scale = self.top_view_scale
        camera_xy = scene['plane_from_camera'][:2, 3]
        robot_xy = invert_transform(scene['base_from_plane'])[:2, 3]
        center = (camera_xy + robot_xy) / 2.0

        def px(xy):
            return (int(round(width / 2 - (xy[1] - center[1]) * scale)),
                    int(round(height / 2 - (xy[0] - center[0]) * scale)))

        image = np.full((height, width, 3), 245, np.uint8)
        half_w, half_h = width / (2 * scale), height / (2 * scale)
        for x in np.arange(math.floor((center[0] - half_h) * 10) / 10, center[0] + half_h, 0.1):
            cv2.line(image, px((x, center[1] + half_w)), px((x, center[1] - half_w)), (220, 220, 220), 1)
        for y in np.arange(math.floor((center[1] - half_w) * 10) / 10, center[1] + half_w, 0.1):
            cv2.line(image, px((center[0] + half_h, y)), px((center[0] - half_h, y)), (220, 220, 220), 1)

        cv2.circle(image, px(robot_xy), int(FR3_REACH * scale), (200, 200, 200), 1, cv2.LINE_AA)
        cv2.circle(image, px(robot_xy), int(0.08 * scale), (90, 90, 90), -1)
        put_text(image, self.base_frame, (px(robot_xy)[0] + 22, px(robot_xy)[1]), (90, 90, 90), 0.45)
        cv2.arrowedLine(image, px((0, 0)), px((0.1, 0)), (0, 0, 220), 2, tipLength=0.2)
        cv2.arrowedLine(image, px((0, 0)), px((0, 0.1)), (0, 160, 0), 2, tipLength=0.2)
        put_text(image, self.plane_frame, (px((0, 0))[0] + 6, px((0, 0))[1] + 16), (60, 60, 60), 0.4)
        cv2.drawMarker(image, px(camera_xy), (60, 60, 60), cv2.MARKER_TRIANGLE_UP, 14, 2)
        put_text(image, 'camera', (px(camera_xy)[0] + 10, px(camera_xy)[1] + 5), (60, 60, 60), 0.45)
        if scene['tool'] is not None:
            cv2.drawMarker(image, px(scene['tool']), (180, 0, 180), cv2.MARKER_TILTED_CROSS, 14, 2)
            put_text(image, 'tool', (px(scene['tool'])[0] + 10, px(scene['tool'])[1]), (180, 0, 180), 0.45)

        located = [hand for hand in hands if 'box_plane' in hand]
        footprints = [np.array([px(p) for p in hand['box_plane'][:4]], np.int32)
                      for hand in located]
        overlay = image.copy()
        for hand, footprint in zip(located, footprints):
            cv2.fillPoly(overlay, [footprint], bgr(hand['side']))
        image = cv2.addWeighted(overlay, 0.25, image, 0.75, 0.0)
        for hand, footprint in zip(located, footprints):
            color = bgr(hand['side'])
            cv2.polylines(image, [footprint], True, color, 2, cv2.LINE_AA)
            landmarks = hand['landmarks_plane']
            for a, b in HAND_CONNECTIONS:
                if np.isfinite(landmarks[[a, b]]).all():
                    cv2.line(image, px(landmarks[a]), px(landmarks[b]), color, 1, cv2.LINE_AA)
            text = hand['side'] + (
                f" tool {hand['tool_distance']:.2f} m" if 'tool_distance' in hand else '')
            put_text(image, text, (footprint[:, 0].min(), footprint[:, 1].min() - 6), color, 0.45)

        bar = (12, height - 14)
        cv2.line(image, bar, (bar[0] + int(0.1 * scale), bar[1]), (0, 0, 0), 2)
        put_text(image, '10 cm', (bar[0], bar[1] - 6), (0, 0, 0), 0.4)
        put_text(image, 'top view (+x up, +y left)', (8, 20), (0, 0, 0), 0.5)

        top_view = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        top_view.header.stamp = msg.header.stamp
        top_view.header.frame_id = self.plane_frame
        self.top_view_pub.publish(top_view)

    def destroy_node(self):
        self.landmarker.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
