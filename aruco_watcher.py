"""Background ArUco marker detection.

The watcher only *reports* markers; it never talks to the flight controller.
The mission flow decides what to do, which keeps all drone commands on one
thread.

While `enabled` is set, every frame with a valid marker becomes a "hit":
    {"id", "frac" (share of the frame), "center" (u, v), "lat", "lon", "time"}
lat/lon are worked out from the drone pose and the camera model (camera
fixed to the frame, looking straight down) and are None without a pose.
`detected` is set after `consecutive_frames` hits in a row.
"""
import collections
import os
import threading
import time

import cv2

from camera_model import CameraModel
from mission_common import CameraError, log


def _make_detector(dictionary_name):
    """Return detect(gray) -> (corners, ids), for old and new OpenCV."""
    aruco = cv2.aruco
    dict_id = getattr(aruco, dictionary_name)
    if hasattr(aruco, "getPredefinedDictionary"):
        dictionary = aruco.getPredefinedDictionary(dict_id)
    else:  # very old OpenCV
        dictionary = aruco.Dictionary_get(dict_id)
    # On OpenCV < 4.7 the DetectorParameters() constructor exists but gives a
    # broken object that segfaults; DetectorParameters_create() is the right call.
    if hasattr(aruco, "DetectorParameters_create"):
        params = aruco.DetectorParameters_create()
    else:
        params = aruco.DetectorParameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR

    if hasattr(aruco, "ArucoDetector"):  # OpenCV >= 4.7
        detector = aruco.ArucoDetector(dictionary, params)

        def detect(gray):
            corners, ids, _ = detector.detectMarkers(gray)
            return corners, ids
    else:
        def detect(gray):
            corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=params)
            return corners, ids
    return detect


class _Camera:
    """'picamera2' (Pi camera), 'sim' (SITL), or a number (USB/laptop webcam)."""

    def __init__(self, cfg, pose_fn, model):
        self.source = str(cfg["source"])
        size = tuple(cfg["frame_size"])
        if self.source == "picamera2":
            from picamera2 import Picamera2
            self.cam = Picamera2()
            # Picamera2's "RGB888" arrays are in BGR order, which is what OpenCV expects.
            self.cam.configure(self.cam.create_preview_configuration(main={"format": "RGB888", "size": size}))
            self.cam.start()
        elif self.source == "sim":
            from sim_camera import SimCamera
            self.cam = SimCamera(cfg["sim_world"], pose_fn, model, cfg["dictionary"])
        else:
            self.cam = cv2.VideoCapture(int(self.source))
            if not self.cam.isOpened():
                raise CameraError(f"could not open camera {self.source}")

    def read(self):
        if self.source in ("picamera2",):
            return self.cam.capture_array()
        if self.source == "sim":
            return self.cam.read()
        ok, frame = self.cam.read()
        return frame if ok else None

    def close(self):
        try:
            if self.source == "picamera2":
                self.cam.stop()
                self.cam.close()
            elif self.source == "sim":
                self.cam.close()
            else:
                self.cam.release()
        except Exception:
            pass


class ArucoWatcher:
    def __init__(self, aruco_cfg, pose_fn=None):
        self.cfg = aruco_cfg
        self.pose_fn = pose_fn
        w, h = aruco_cfg["frame_size"]
        self.model = CameraModel(w, h, aruco_cfg.get("hfov_deg", 62.2))
        self.allowed_ids = set(aruco_cfg.get("allowed_ids") or [])
        self.debug_dir = aruco_cfg.get("debug_dir")
        self.enabled = threading.Event()   # set by the mission while searching
        self.detected = threading.Event()  # set here once a marker is confirmed
        self.failed = threading.Event()    # set here if the camera dies
        self.last = None                   # the hit that confirmed the detection
        self._hits = collections.deque(maxlen=200)
        self._hits_lock = threading.Lock()
        self._detect = _make_detector(aruco_cfg["dictionary"])
        self._camera = None
        self._stop = threading.Event()
        self._thread = None
        self._saved = 0

    def start(self):
        """Open the camera and check it gives frames. Raises CameraError."""
        try:
            self._camera = _Camera(self.cfg, self.pose_fn, self.model)
        except CameraError:
            raise
        except Exception as e:
            raise CameraError(f"camera failed to start: {e}") from e
        for _ in range(30):
            if self._camera.read() is not None:
                break
            time.sleep(0.1)
        else:
            self._camera.close()
            raise CameraError("camera started but returned no frames")
        if self.debug_dir:
            os.makedirs(self.debug_dir, exist_ok=True)
        self._thread = threading.Thread(target=self._run, name="aruco", daemon=True)
        self._thread.start()
        log(f"Camera OK ({self.cfg['source']}), marker watcher running.")

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        if self._camera is not None:
            self._camera.close()

    def clear_hits(self):
        with self._hits_lock:
            self._hits.clear()

    def hits(self):
        with self._hits_lock:
            return list(self._hits)

    def find_marker(self, frame, pose=None):
        """Return the biggest valid marker in the frame as a hit dict, or None."""
        if frame.ndim == 3:
            code = cv2.COLOR_BGRA2GRAY if frame.shape[2] == 4 else cv2.COLOR_BGR2GRAY
            gray = cv2.cvtColor(frame, code)
        else:
            gray = frame
        corners, ids = self._detect(gray)
        if ids is None:
            return None
        h, w = gray.shape[:2]
        best = None
        for c, marker_id in zip(corners, ids.flatten()):
            marker_id = int(marker_id)
            if self.allowed_ids and marker_id not in self.allowed_ids:
                continue
            pts = c.reshape(-1, 2)
            frac = cv2.contourArea(pts) / float(w * h)
            if frac >= self.cfg["min_area_fraction"] and (best is None or frac > best["frac"]):
                cu, cv_ = (float(x) for x in pts.mean(axis=0))
                best = {"id": marker_id, "frac": frac, "center": (cu, cv_),
                        "lat": None, "lon": None, "time": time.time(), "corners": pts}
        if best is not None and pose is not None:
            ll = self.model.pixel_to_latlon(best["center"][0], best["center"][1], pose)
            if ll is not None:
                best["lat"], best["lon"] = ll
        return best

    def _save(self, frame, hit, tag):
        if not self.debug_dir or self._saved >= 40:
            return
        img = frame.copy()
        pts = hit["corners"].astype(int).reshape(-1, 1, 2)
        cv2.polylines(img, [pts], True, (0, 0, 255), 2)
        cv2.putText(img, f"{tag} id {hit['id']}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imwrite(os.path.join(self.debug_dir, f"{self._saved:02d}_{tag}.jpg"), img)
        self._saved += 1

    def _run(self):
        consecutive, bad_frames = 0, 0
        while not self._stop.is_set():
            try:
                frame = self._camera.read()
            except Exception as e:
                log(f"Camera error: {e}")
                self.failed.set()
                return
            if frame is None:
                bad_frames += 1
                if bad_frames > 50:
                    log("Camera stopped returning frames.")
                    self.failed.set()
                    return
                time.sleep(0.05)
                continue
            bad_frames = 0
            if not self.enabled.is_set():
                consecutive = 0
                if self.cfg.get("source") != "sim":
                    time.sleep(0.05)
                continue
            pose = self.pose_fn() if self.pose_fn else None
            hit = self.find_marker(frame, pose)
            if hit is None:
                consecutive = 0
                continue
            with self._hits_lock:
                self._hits.append(hit)
            consecutive += 1
            if consecutive >= self.cfg["consecutive_frames"] and not self.detected.is_set():
                self.last = hit
                self.detected.set()
                self._save(frame, hit, "detected")
            elif self.detected.is_set() and consecutive % 15 == 0:
                self._save(frame, hit, "tracking")
