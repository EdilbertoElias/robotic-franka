"""ArUco helpers that work with both the OpenCV < 4.7 and >= 4.7 APIs."""
import cv2
import numpy as np


def get_dictionary(name):
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def draw_marker(dictionary, marker_id, side_pixels):
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, side_pixels)
    return cv2.aruco.drawMarker(dictionary, marker_id, side_pixels)


def detect_markers(gray, dictionary):
    """Return {marker_id: (4, 2) corners} ordered TL, TR, BR, BL."""
    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=cv2.aruco.DetectorParameters_create())
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2) for i, c in zip(ids.ravel(), corners)}


def marker_object_points(size):
    """Marker corners in the marker frame: origin at the center, x right, y up, z out."""
    h = size / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


def estimate_marker_pose(corners, size, camera_matrix, dist_coeffs):
    """Return (camera_from_marker 4x4, reprojection error in px) or None."""
    object_points = marker_object_points(size)
    ok, rvec, tvec = cv2.solvePnP(
        object_points, corners.astype(np.float64), camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - corners) ** 2, axis=1))))
    transform = np.eye(4)
    transform[:3, :3] = cv2.Rodrigues(rvec)[0]
    transform[:3, 3] = tvec.ravel()
    return transform, error
