"""A fake downward camera for SITL: renders the ground and ArUco markers from
the drone's live GPS position and attitude.

Selected with aruco.source = "sim"; the world is described in aruco.sim_world:
    "markers": [{"id": 7, "lat": .., "lon": .., "size_m": 1.5}, ...]
    "ground_seed", "fps", "noise", "save_dir", "save_every_s"
"""
import os
import time

import cv2
import numpy as np

from camera_model import offset_ne

GROUND_RES_M = 0.05      # metres per pixel of the ground texture
GROUND_TILE_PX = 1024    # tiled forever (about 51 m square)
MARKER_TEX_PX = 480      # texture size of one marker incl. white border
QUIET_ZONE = 0.25        # white border around the marker, fraction of marker size


def _dictionary(name):
    aruco = cv2.aruco
    dict_id = getattr(aruco, name)
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(dict_id)
    return aruco.Dictionary_get(dict_id)


def _marker_image(dictionary, marker_id, px):
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, px)
    return cv2.aruco.drawMarker(dictionary, marker_id, px)


def _ground_tile(seed):
    """Grass with dirt patches, paths and a few rooftops."""
    rng = np.random.default_rng(seed)
    n = GROUND_TILE_PX
    low = cv2.resize(rng.random((16, 16)).astype(np.float32), (n, n), interpolation=cv2.INTER_CUBIC)
    mid = cv2.resize(rng.random((96, 96)).astype(np.float32), (n, n), interpolation=cv2.INTER_LINEAR)
    fine = rng.random((n, n)).astype(np.float32)
    t = np.clip(0.5 * low + 0.3 * mid + 0.2 * fine, 0, 1)[..., None]
    grass = np.array([40, 120, 60], np.float32)   # BGR
    dirt = np.array([70, 110, 140], np.float32)
    img = grass * (1 - t) + dirt * t
    img = np.clip(img, 0, 255).astype(np.uint8)
    for _ in range(4):
        p1 = tuple(int(v) for v in rng.integers(0, n, 2))
        p2 = tuple(int(v) for v in rng.integers(0, n, 2))
        cv2.line(img, p1, p2, (150, 160, 170), int(rng.integers(20, 50)))
    for _ in range(3):
        x, y = (int(v) for v in rng.integers(0, n - 200, 2))
        w, h = (int(v) for v in rng.integers(80, 200, 2))
        cv2.rectangle(img, (x, y), (x + w, y + h), (90, 90, 160), -1)
    return img


class SimCamera:
    def __init__(self, world, pose_fn, camera_model, dictionary_name):
        if pose_fn is None:
            raise ValueError("the sim camera needs the vehicle pose")
        self.pose_fn = pose_fn
        self.cam = camera_model
        self.world = world
        self.period = 1.0 / world.get("fps", 30)
        self.noise = world.get("noise", 6)
        self.rng = np.random.default_rng(0)
        self.ground = _ground_tile(world.get("ground_seed", 1))
        self.origin = None   # world lat/lon that the ground tile is anchored to
        self.markers = []
        dictionary = _dictionary(dictionary_name)
        border = int(MARKER_TEX_PX * QUIET_ZONE / (1 + 2 * QUIET_ZONE))
        inner = MARKER_TEX_PX - 2 * border
        for m in world["markers"]:
            tex = np.full((MARKER_TEX_PX, MARKER_TEX_PX), 255, np.uint8)
            tex[border:border + inner, border:border + inner] = _marker_image(dictionary, m["id"], inner)
            self.markers.append({
                "lat": m["lat"], "lon": m["lon"],
                "size_total": m["size_m"] * (1 + 2 * QUIET_ZONE),
                "tex": cv2.cvtColor(tex, cv2.COLOR_GRAY2BGR),
            })
        self.save_dir = world.get("save_dir")
        self.save_every = world.get("save_every_s", 2.0)
        self._last_save = 0.0
        self._next_frame = 0.0
        if self.save_dir:
            os.makedirs(self.save_dir, exist_ok=True)

    def read(self):
        now = time.time()
        if now < self._next_frame:
            time.sleep(self._next_frame - now)
        self._next_frame = max(now, self._next_frame) + self.period

        pose = self.pose_fn()
        if pose is None:
            return None
        frame = self.render_pose(pose)
        alt, yaw = max(pose[2], 0.3), pose[5]

        if self.save_dir and now - self._last_save >= self.save_every:
            self._last_save = now
            label = f"alt {alt:5.1f} m  yaw {np.degrees(yaw):5.0f}"
            shot = frame.copy()
            cv2.putText(shot, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imwrite(os.path.join(self.save_dir, time.strftime("%H%M%S") + f"_{int(now * 10) % 10}.jpg"), shot)
        return frame

    def render_pose(self, pose, noise=True):
        """The downward camera image for pose = (lat, lon, rel_alt, roll, pitch, yaw)."""
        lat, lon, alt, roll, pitch, yaw = pose
        alt = max(alt, 0.3)  # lens height when sitting on the ground
        h = self.cam.ground_to_pixel_h(alt, roll, pitch, yaw)
        frame = self.render_view(lat, lon, h, self.cam.size, marker_radius_m=3 * alt + 20)
        if noise and self.noise:
            frame = cv2.GaussianBlur(frame, (3, 3), 0)
            n = self.rng.normal(0, self.noise, frame.shape).astype(np.int16)
            frame = np.clip(frame.astype(np.int16) + n, 0, 255).astype(np.uint8)
        return frame

    def render_view(self, lat, lon, h, size, marker_radius_m=60):
        """Render the ground and markers for any camera.

        h maps (north, east, 1) in metres from the point (lat, lon) to pixels.
        """
        if self.origin is None:
            self.origin = (lat, lon)
        w_px, h_px = size
        # Ground: tile pixel (x, y) -> metres from (lat, lon).
        on, oe = offset_ne(lat, lon, *self.origin)
        r = GROUND_RES_M
        a_ground = np.array([[0, -r, on], [r, 0, oe], [0, 0, 1.0]])
        frame = cv2.warpPerspective(self.ground, h @ a_ground, (w_px, h_px),
                                    flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
        for m in self.markers:
            mn, me = offset_ne(lat, lon, m["lat"], m["lon"])
            if mn * mn + me * me > marker_radius_m ** 2:
                continue
            s = m["size_total"] / MARKER_TEX_PX
            half = m["size_total"] / 2
            # Marker texture (x, y) -> metres; top of the marker faces north.
            a_marker = np.array([[0, -s, mn + half], [s, 0, me - half], [0, 0, 1.0]])
            hm = h @ a_marker
            tex = cv2.warpPerspective(m["tex"], hm, (w_px, h_px), flags=cv2.INTER_LINEAR)
            mask = cv2.warpPerspective(np.full(m["tex"].shape[:2], 255, np.uint8), hm, (w_px, h_px),
                                       flags=cv2.INTER_LINEAR)
            alpha = (mask.astype(np.float32) / 255.0)[..., None]
            frame = (frame * (1 - alpha) + tex * alpha).astype(np.uint8)
        return frame

    def close(self):
        pass
