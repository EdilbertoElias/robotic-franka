import math

import cv2
import numpy as np


def quaternion_from_matrix(rotation):
    """Return (x, y, z, w) for a 3x3 rotation matrix."""
    m = np.asarray(rotation, dtype=float)
    trace = np.trace(m)
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def matrix_from_quaternion(q):
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def optical_frame_rotation(yaw, pitch_down):
    """Rotation of a camera optical frame (z forward, x right, y down) whose
    viewing direction has the given yaw about +z and is tilted pitch_down
    below the horizontal plane of the parent frame."""
    forward = np.array([
        math.cos(pitch_down) * math.cos(yaw),
        math.cos(pitch_down) * math.sin(yaw),
        -math.sin(pitch_down)])
    right = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
    down = np.cross(forward, right)
    return np.column_stack([right, down, forward])


def make_transform(rotation, translation):
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def invert_transform(transform):
    rotation = transform[:3, :3]
    return make_transform(rotation.T, -rotation.T @ transform[:3, 3])


def transform_from_msg(msg):
    """geometry_msgs/Transform -> 4x4 matrix."""
    t, r = msg.translation, msg.rotation
    return make_transform(
        matrix_from_quaternion([r.x, r.y, r.z, r.w]), [t.x, t.y, t.z])


def average_quaternions(quaternions):
    """Sign-aligned mean; adequate for samples that are close to each other."""
    qs = np.array(quaternions, dtype=float)
    qs[np.sum(qs * qs[0], axis=1) < 0.0] *= -1.0
    mean = qs.mean(axis=0)
    return mean / np.linalg.norm(mean)


def pixel_rays(pixels, camera_matrix, dist_coeffs):
    """Undistorted viewing rays (z = 1) in the camera optical frame."""
    points = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
    normalized = cv2.undistortPoints(points, camera_matrix, dist_coeffs).reshape(-1, 2)
    return np.column_stack([normalized, np.ones(len(normalized))])


def intersect_rays_with_plane(plane_from_camera, rays, height):
    """Intersect camera rays with the plane z = height of the plane frame.

    Returns an (N, 3) array in the plane frame; rows are NaN where the ray is
    parallel to the plane or the intersection lies behind the camera.
    """
    origin = plane_from_camera[:3, 3]
    directions = rays @ plane_from_camera[:3, :3].T
    points = np.full((len(directions), 3), np.nan)
    dz = directions[:, 2]
    valid = np.abs(dz) > 1e-9
    scale = np.full(len(directions), np.nan)
    scale[valid] = (height - origin[2]) / dz[valid]
    valid &= scale > 0.0
    points[valid] = origin + scale[valid, None] * directions[valid]
    return points


def hand_zone(points_xy, margin):
    """Smallest oriented rectangle around the points (top view), grown by margin.

    Returns (center (2,), size (2,), yaw, corners (4, 2) counter-clockwise).
    """
    (cx, cy), (width, height), angle = cv2.minAreaRect(np.asarray(points_xy, dtype=np.float32))
    yaw = math.radians(angle)
    size = np.array([width, height]) + 2.0 * margin
    axes = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    unit = np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]])
    corners = np.array([cx, cy]) + (unit * size) @ axes.T
    return np.array([cx, cy]), size, yaw, corners


def distance_to_zone(point, corners_xy, z_min, z_max):
    """Distance from a 3D point to the prism with footprint corners_xy between
    z_min and z_max (all in the plane frame); 0 when the point is inside."""
    contour = np.asarray(corners_xy, dtype=np.float32).reshape(-1, 1, 2)
    signed = cv2.pointPolygonTest(contour, (float(point[0]), float(point[1])), True)
    horizontal = max(0.0, -signed)
    vertical = max(0.0, z_min - point[2], point[2] - z_max)
    return math.hypot(horizontal, vertical)


class OneEuroFilter:
    """One Euro filter (Casiez et al., 2012) for a vector signal.

    The filter's internal derivative (raw sample minus the lagging filtered
    value) only adapts the cutoff; it overestimates speed on ramps. The
    returned velocity is instead the low-passed derivative of the filtered
    position, which is unbiased for constant-velocity motion.
    """

    def __init__(self, min_cutoff=1.0, beta=0.5, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.reset()

    def reset(self):
        self.t = None
        self.x = None
        self.dx = None
        self.velocity = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, t, x):
        """Return (filtered value, velocity estimate)."""
        x = np.asarray(x, dtype=float)
        if self.t is None:
            self.t, self.x = t, x
            self.dx = np.zeros_like(x)
            self.velocity = np.zeros_like(x)
            return self.x, self.velocity
        if t <= self.t:
            return self.x, self.velocity
        dt = t - self.t
        a_d = self._alpha(self.d_cutoff, dt)
        self.dx = a_d * (x - self.x) / dt + (1.0 - a_d) * self.dx
        cutoff = self.min_cutoff + self.beta * np.linalg.norm(self.dx)
        a = self._alpha(cutoff, dt)
        previous = self.x
        self.x = a * x + (1.0 - a) * self.x
        self.velocity = a_d * (self.x - previous) / dt + (1.0 - a_d) * self.velocity
        self.t = t
        return self.x, self.velocity
