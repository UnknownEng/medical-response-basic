"""Helpers shared by mission_pymavlink.py and mission_ros1.py.

No drone or camera libraries are imported here, so this module (and
mission_flow.py) can be tested on any computer.
"""
import json
import math
import os
import sys
import threading
import time
from collections import namedtuple

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(HERE, "mission_config.json")

# MAVLink frames whose x/y are latitude/longitude.
GLOBAL_FRAMES = {0, 3, 5, 6, 10, 11}
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_LAND = 21


class Abort(Exception):
    """Stop the mission. `action` is what to command if the drone is armed:
    "RTL", "LAND" or "NONE" (leave the flight controller alone)."""

    def __init__(self, reason, action="RTL"):
        super().__init__(reason)
        self.action = action


class PilotOverride(Exception):
    """The flight mode changed without us (RC pilot or an ArduPilot failsafe).
    The script stops sending commands and leaves control to them."""


class CameraError(RuntimeError):
    pass


_log_lock = threading.Lock()


def log(msg):
    # One write per line under a lock, so lines from different threads never merge.
    with _log_lock:
        sys.stdout.write(f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        sys.stdout.flush()


# ---------------------------------------------------------------- config

def load_config(path=None):
    path = os.path.expanduser(path or DEFAULT_CONFIG)
    with open(path, "r") as f:
        cfg = json.load(f)
    base = os.path.dirname(os.path.abspath(path))

    def resolve(p):  # "~/x" and absolute paths as-is, others relative to the config file
        return p if not p else os.path.join(base, os.path.expanduser(p))

    cfg["mission"]["waypoint_file"] = resolve(cfg["mission"]["waypoint_file"])
    a = cfg["aruco"]
    a["debug_dir"] = resolve(a.get("debug_dir"))
    if a.get("sim_world"):
        a["sim_world"]["save_dir"] = resolve(a["sim_world"].get("save_dir"))
    validate_config(cfg)
    return cfg


def validate_config(cfg):
    errors = []
    t, p, m, a = cfg["target"], cfg["payload"], cfg["mission"], cfg["aruco"]
    if not -90 <= t["lat"] <= 90 or not -180 <= t["lon"] <= 180:
        errors.append("target lat/lon out of range")
    for key in ("alt_m", "drop_alt_m"):
        if not 2 <= t[key] <= 120:
            errors.append(f"target.{key} must be between 2 and 120 m")
    if not 2 <= m["takeoff_alt_m"] <= 120:
        errors.append("mission.takeoff_alt_m must be between 2 and 120 m")
    if t["arrival_radius_m"] <= 0:
        errors.append("target.arrival_radius_m must be > 0")
    if not 1 <= p["servo_channel"] <= 16:
        errors.append("payload.servo_channel must be 1-16")
    for key in ("pwm_at_marker", "pwm_at_target"):
        if not 800 <= p[key] <= 2200:
            errors.append(f"payload.{key} must be 800-2200")
    if a["consecutive_frames"] < 1:
        errors.append("aruco.consecutive_frames must be >= 1")
    if errors:
        raise ValueError("Bad mission_config.json: " + "; ".join(errors))


# ---------------------------------------------------------------- waypoints

WaypointItem = namedtuple(
    "WaypointItem",
    "seq current frame command p1 p2 p3 p4 x y z autocontinue",
)


def parse_qgc_wpl(path):
    """Parse a QGroundControl / Mission Planner 'QGC WPL 110' file."""
    with open(path, "r") as f:
        lines = [ln.strip() for ln in f]
    if not lines or not lines[0].startswith("QGC WPL"):
        raise ValueError(f"{path}: not a 'QGC WPL' waypoint file")
    items = []
    for lineno, line in enumerate(lines[1:], start=2):
        if not line:
            continue
        parts = line.split()
        if len(parts) < 12:
            raise ValueError(f"{path}:{lineno}: expected 12 columns, got {len(parts)}")
        try:
            seq, current, frame, command = (int(v) for v in parts[:4])
            p1, p2, p3, p4, x, y, z = (float(v) for v in parts[4:11])
            autocontinue = int(float(parts[11]))
        except ValueError as e:
            raise ValueError(f"{path}:{lineno}: {e}") from None
        if seq != len(items):
            raise ValueError(f"{path}:{lineno}: sequence {seq} should be {len(items)}")
        items.append(WaypointItem(seq, current, frame, command, p1, p2, p3, p4, x, y, z, autocontinue))
    if len(items) < 2:
        raise ValueError(f"{path}: needs a home item plus at least one waypoint")
    return items


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(h))
