import math

import cv2
import numpy as np

from aided_building_hand_perception.aruco import (
    detect_markers, draw_marker, estimate_marker_pose, get_dictionary, marker_object_points)
from aided_building_hand_perception.geometry import (
    OneEuroFilter, average_quaternions, distance_to_zone, hand_zone, intersect_rays_with_plane,
    invert_transform, make_transform, matrix_from_quaternion, matrix_from_rpy,
    optical_frame_rotation, pixel_rays, quaternion_from_matrix)

# Intrinsics of the notebook webcam calibration (640x480)
K = np.array([[591.47, 0.0, 320.13], [0.0, 591.70, 254.45], [0.0, 0.0, 1.0]])
D = np.array([-0.175156, 0.28136, 0.003332, 0.000487, 0.0])


def camera_pose():
    """Camera 0.35 m behind and 0.30 m above the plane origin, tilted 30 deg down."""
    return make_transform(optical_frame_rotation(0.0, math.radians(30.0)), [-0.35, 0.0, 0.30])


def test_quaternion_round_trip():
    rng = np.random.default_rng(0)
    for _ in range(200):
        rotation = matrix_from_rpy(*rng.uniform(-math.pi, math.pi, 3))
        q = quaternion_from_matrix(rotation)
        assert np.allclose(matrix_from_quaternion(q), rotation, atol=1e-9)


def test_average_quaternions_handles_sign_flip():
    q = quaternion_from_matrix(matrix_from_rpy(0.1, 0.2, 0.3))
    mean = average_quaternions([q, -q, q])
    assert np.allclose(np.abs(np.dot(mean, q)), 1.0)


def test_optical_frame_rotation():
    rotation = optical_frame_rotation(0.0, math.radians(90.0))
    assert np.isclose(np.linalg.det(rotation), 1.0)
    assert np.allclose(rotation[:, 2], [0.0, 0.0, -1.0], atol=1e-9)  # looks straight down
    level = optical_frame_rotation(math.radians(90.0), 0.0)
    assert np.allclose(level[:, 2], [0.0, 1.0, 0.0], atol=1e-9)
    assert np.allclose(level[:, 1], [0.0, 0.0, -1.0], atol=1e-9)  # image down = world down


def test_back_projection_recovers_plane_points():
    plane_from_camera = camera_pose()
    camera_from_plane = invert_transform(plane_from_camera)
    height = 0.02
    points = np.array([[x, y, height] for x in (-0.1, 0.0, 0.15, 0.3) for y in (-0.12, 0.0, 0.12)])

    rvec = cv2.Rodrigues(camera_from_plane[:3, :3])[0]
    pixels, _ = cv2.projectPoints(points, rvec, camera_from_plane[:3, 3], K, D)
    pixels = pixels.reshape(-1, 2)
    assert np.all((pixels >= 0) & (pixels < [640, 480]))

    recovered = intersect_rays_with_plane(plane_from_camera, pixel_rays(pixels, K, D), height)
    assert np.allclose(recovered, points, atol=1e-4)


def test_rays_above_horizon_do_not_hit_the_table():
    plane_from_camera = camera_pose()
    # A ray pointing 45 deg upwards in the camera frame (y is down)
    rays = np.array([[0.0, -1.0, 1.0]])
    assert np.isnan(intersect_rays_with_plane(plane_from_camera, rays, 0.0)).all()


def test_hand_zone_contains_points_with_margin():
    rng = np.random.default_rng(1)
    yaw = math.radians(35.0)
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    # Hand-like cloud: 18 x 8 cm, rotated 35 deg, centered at (0.1, -0.2)
    points = (rng.uniform([-0.09, -0.04], [0.09, 0.04], (21, 2)) @ rotation.T) + [0.1, -0.2]
    center, size, _, corners = hand_zone(points, margin=0.03)

    assert np.allclose(center, [0.1, -0.2], atol=0.02)
    # tight: never larger than the sampled extent plus the margin on both sides
    assert np.all(np.array(sorted(size)) <= np.array([0.08, 0.18]) + 0.06 + 1e-6)
    contour = corners.astype(np.float32).reshape(-1, 1, 2)
    for p in points:
        # every landmark is inside, at least `margin` away from the border
        assert cv2.pointPolygonTest(contour, (float(p[0]), float(p[1])), True) >= 0.03 - 1e-6
    edges = np.linalg.norm(np.roll(corners, -1, axis=0) - corners, axis=1)
    assert np.allclose(sorted(edges), sorted([size[0], size[1], size[0], size[1]]), atol=1e-6)


def test_distance_to_zone():
    corners = np.array([[0.0, 0.0], [0.1, 0.0], [0.1, 0.1], [0.0, 0.1]])
    assert distance_to_zone([0.05, 0.05, 0.05], corners, 0.0, 0.1) == 0.0
    assert math.isclose(distance_to_zone([0.3, 0.05, 0.05], corners, 0.0, 0.1), 0.2, abs_tol=1e-6)
    assert math.isclose(distance_to_zone([0.05, 0.05, 0.4], corners, 0.0, 0.1), 0.3, abs_tol=1e-6)
    assert math.isclose(distance_to_zone([0.4, 0.05, 0.5], corners, 0.0, 0.1), 0.5, abs_tol=1e-6)


def test_one_euro_filter_tracks_velocity():
    one_euro = OneEuroFilter(min_cutoff=1.0, beta=2.0)
    velocity = np.array([0.2, -0.1, 0.0])
    for i in range(90):
        t = i / 30.0
        position, estimated = one_euro(t, velocity * t)
    assert np.allclose(estimated, velocity, atol=0.005)
    # Smoothing trades a small lag: ~1.5 cm at 0.22 m/s with these settings
    assert np.linalg.norm(position - velocity * t) < 0.02


def test_aruco_pose_from_synthetic_image():
    size = 0.10
    dictionary = get_dictionary('DICT_4X4_50')
    # 300 px marker = 6 cells of 50 px; quiet zone of 2 cells, as in generate_marker
    texture = np.full((500, 500), 255, np.uint8)
    texture[100:400, 100:400] = draw_marker(dictionary, 0, 300)

    plane_from_camera = camera_pose()
    camera_from_plane = invert_transform(plane_from_camera)
    rvec = cv2.Rodrigues(camera_from_plane[:3, :3])[0]
    tvec = camera_from_plane[:3, 3]

    # Texture corners (with the white quiet zone) placed on the table, marker centered at the origin
    half = size / 2.0 * 500.0 / 300.0
    plane_corners = np.array([[-half, half, 0.0], [half, half, 0.0],
                              [half, -half, 0.0], [-half, -half, 0.0]])
    image_corners, _ = cv2.projectPoints(plane_corners, rvec, tvec, K, np.zeros(5))
    homography = cv2.getPerspectiveTransform(
        np.float32([[0, 0], [499, 0], [499, 499], [0, 499]]),
        np.float32(image_corners.reshape(4, 2)))
    image = cv2.warpPerspective(texture, homography, (640, 480), borderValue=200)

    corners = detect_markers(image, dictionary)[0]
    expected, _ = cv2.projectPoints(marker_object_points(size), rvec, tvec, K, np.zeros(5))
    assert np.allclose(corners, expected.reshape(4, 2), atol=1.5)

    estimated, error = estimate_marker_pose(corners, size, K, np.zeros(5))
    assert error < 1.0
    recovered = invert_transform(estimated)
    assert np.linalg.norm(recovered[:3, 3] - plane_from_camera[:3, 3]) < 0.01
    angle = np.degrees(np.arccos(np.clip(
        (np.trace(recovered[:3, :3].T @ plane_from_camera[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)))
    assert angle < 1.0
