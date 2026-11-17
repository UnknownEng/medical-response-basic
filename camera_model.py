"""Geometry for a camera fixed to the drone, looking straight down.

Conventions: world is local NED (north, east, down) with its origin on the
ground directly below the drone; body is FRD (forward, right, down); the top
of the image points at the drone's nose.

Used both ways:
  * aruco_watcher: marker pixel + drone pose -> marker GPS position
  * sim_camera:    drone pose -> image of the simulated ground
"""
import math

import numpy as np

EARTH_R = 6378137.0

# Camera axes (x right, y down the image, z out of the lens) in body FRD.
R_BODY_CAM = np.array([[0.0, -1.0, 0.0],
                       [1.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0]])


def offset_ne(lat0, lon0, lat, lon):
    """Metres north/east of (lat0, lon0). Fine for distances of a few km."""
    n = math.radians(lat - lat0) * EARTH_R
    e = math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
    return n, e


def add_ne(lat0, lon0, n, e):
    lat = lat0 + math.degrees(n / EARTH_R)
    lon = lon0 + math.degrees(e / (EARTH_R * math.cos(math.radians(lat0))))
    return lat, lon


def rot_body_to_ned(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


class CameraModel:
    def __init__(self, width, height, hfov_deg):
        f = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        self.K = np.array([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1.0]])
        self.K_inv = np.linalg.inv(self.K)
        self.size = (int(width), int(height))

    def _r_ned_cam(self, roll, pitch, yaw):
        return rot_body_to_ned(roll, pitch, yaw) @ R_BODY_CAM

    def ground_to_pixel_h(self, alt, roll, pitch, yaw):
        """Homography: (north, east, 1) in metres from the point below the drone -> pixel."""
        r_cam_ned = self._r_ned_cam(roll, pitch, yaw).T
        m = np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, float(alt)]])
        return self.K @ r_cam_ned @ m

    def pixel_to_ground(self, u, v, alt, roll, pitch, yaw):
        """(north, east) metres from the point below the drone, or None above the horizon."""
        ray = self._r_ned_cam(roll, pitch, yaw) @ (self.K_inv @ np.array([u, v, 1.0]))
        if ray[2] <= 1e-6:
            return None
        t = alt / ray[2]
        return float(ray[0] * t), float(ray[1] * t)

    def pixel_to_latlon(self, u, v, pose):
        """pose = (lat, lon, rel_alt, roll, pitch, yaw)."""
        lat, lon, alt, roll, pitch, yaw = pose
        ne = self.pixel_to_ground(u, v, max(alt, 0.1), roll, pitch, yaw)
        return None if ne is None else add_ne(lat, lon, ne[0], ne[1])
