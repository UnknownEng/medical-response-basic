"""Runs the full mission against a simulated drone (no hardware, no ROS).

    python -m unittest discover -s tests -v
"""
import copy
import json
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from mission_common import (  # noqa: E402
    CameraError, haversine_m, parse_qgc_wpl, validate_config,
)
from mission_flow import DeliveryMission, run_mission  # noqa: E402

with open(os.path.join(ROOT, "mission_config.json")) as f:
    BASE_CFG = json.load(f)

HOME = (33.6800, 73.0450)
WPL = """QGC WPL 110
0\t1\t0\t16\t0\t0\t0\t0\t33.680000\t73.045000\t0\t1
1\t0\t3\t16\t0\t0\t0\t0\t33.681000\t73.045000\t15\t1
2\t0\t3\t16\t0\t0\t0\t0\t33.681000\t73.046000\t15\t1
3\t0\t3\t20\t0\t0\t0\t0\t0\t0\t0\t1
"""


class SimClock:
    def __init__(self):
        self.t = 0.0
        self.on_tick = []

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += max(dt, 0.01)
        for cb in self.on_tick:
            cb(self.t)


class FakeEvent:
    def __init__(self):
        self.flag = False

    def set(self):
        self.flag = True

    def clear(self):
        self.flag = False

    def is_set(self):
        return self.flag


class FakeWatcher:
    def __init__(self, sim, see_marker_after_s=None, fail_start=False, fail_after_s=None):
        self.enabled, self.detected, self.failed = FakeEvent(), FakeEvent(), FakeEvent()
        self.last = None
        self.sim = sim
        self.see_after = see_marker_after_s
        self.fail_start = fail_start
        self.fail_after = fail_after_s
        self.enabled_since = None
        self.started = self.stopped = False

    def start(self):
        if self.fail_start:
            raise CameraError("no camera")
        self.started = True

    def stop(self):
        self.stopped = True

    def clear_hits(self):
        pass

    def hits(self):
        return [self.last] if self.last else []

    def tick(self, t):
        if self.enabled.is_set():
            self.enabled_since = self.enabled_since if self.enabled_since is not None else t
            if self.see_after is not None and t - self.enabled_since >= self.see_after:
                self.last = {"id": 7, "frac": 0.01, "lat": 33.68105, "lon": 73.04550}
                self.detected.set()
        if self.fail_after is not None and self.enabled_since is not None and t - self.enabled_since >= self.fail_after:
            self.failed.set()


class FakeVehicle:
    """Very small physics: climbs/descends 2 m/s, flies 10 m/s."""

    def __init__(self, sim):
        self.sim = sim
        self._armed, self._mode = False, "STABILIZE"
        self.alt, self.lat, self.lon = 0.0, HOME[0], HOME[1]
        self.target = None
        self.takeoff_alt = None
        self.battery_pct = 100
        self.auto_started = None
        self.reached_set = set()
        self.log = []          # every command, in order
        self.servo_at = []     # (lat, lon, alt, armed) when each servo command was sent
        self.refuse_arm = False
        self.link = True

    # adapter API
    def link_ok(self): return self.link
    def shutdown_requested(self): return False
    @property
    def armed(self): return self._armed
    @property
    def mode(self): return self._mode
    def rel_alt(self): return self.alt
    def position(self): return (self.lat, self.lon)
    def gps_ok(self, n): return True, "sim"
    def battery(self): return self.battery_pct, None
    def reached(self): return set(self.reached_set)
    def clear_reached(self): self.reached_set.clear()

    def set_mode(self, name, timeout=8):
        self.log.append(("mode", name))
        self._mode = name
        if name == "AUTO":
            self.auto_started = self.sim.t
        return True

    def arm(self):
        self.log.append(("arm",))
        if self.refuse_arm:
            return False
        self._armed = True
        return True

    def takeoff(self, alt):
        self.log.append(("takeoff", alt))
        self.takeoff_alt = alt
        return True

    def goto(self, lat, lon, alt):
        self.target = (lat, lon, alt)

    def set_servo(self, ch, pwm):
        self.log.append(("servo", ch, pwm))
        self.servo_at.append((self.lat, self.lon, self.alt, self._armed))
        return True

    def upload_mission(self, items):
        self.log.append(("upload", len(items)))
        return True

    def lock_home_here(self):
        self.log.append(("home",))
        return True

    def tick(self, t, dt=0.2):
        if not self._armed:
            return
        if self._mode == "GUIDED":
            if self.target:
                lat, lon, alt = self.target
                d = haversine_m(self.lat, self.lon, lat, lon)
                step = min(1.0, 10 * dt / d) if d > 0 else 1.0
                self.lat += (lat - self.lat) * step
                self.lon += (lon - self.lon) * step
                self.alt += max(-2 * dt, min(2 * dt, alt - self.alt))
            elif self.takeoff_alt is not None:
                self.alt = min(self.takeoff_alt, self.alt + 2 * dt)
        elif self._mode == "AUTO":
            elapsed = t - self.auto_started
            for seq, at in ((1, 20), (2, 40), (3, 60)):
                if elapsed >= at:
                    self.reached_set.add(seq)
        elif self._mode in ("LAND", "RTL"):
            self.alt = max(0.0, self.alt - 2 * dt)
            if self.alt == 0.0:
                self._armed = False
                self.target = None
                self.takeoff_alt = None


def make(cfg_changes=None, **watcher_kw):
    cfg = copy.deepcopy(BASE_CFG)
    cfg["target"].update({"lat": 33.6820, "lon": 73.0470})
    cfg["ground"]["wait_s"] = 5
    for section, values in (cfg_changes or {}).items():
        cfg[section].update(values)
    sim = SimClock()
    v = FakeVehicle(sim)
    w = FakeWatcher(sim, **watcher_kw)
    sim.on_tick += [v.tick, w.tick]
    with tempfile.NamedTemporaryFile("w", suffix=".waypoints", delete=False) as f:
        f.write(WPL)
    items = parse_qgc_wpl(f.name)
    os.unlink(f.name)
    m = DeliveryMission(v, w, cfg, items, clock=sim, sleep=sim.sleep, ask=lambda _: None)
    return m, v, w


class MissionTests(unittest.TestCase):
    def test_full_delivery(self):
        m, v, w = make(see_marker_after_s=10)
        self.assertEqual(run_mission(m), 0)
        modes = [e[1] for e in v.log if e[0] == "mode"]
        self.assertEqual(modes, ["GUIDED", "AUTO", "GUIDED", "LAND", "GUIDED", "RTL"])
        # landed on the marker (centering step), not where it was first seen
        lat, lon, alt, armed = v.servo_at[0]
        self.assertLessEqual(haversine_m(lat, lon, 33.68105, 73.04550), 0.5)
        self.assertEqual([e for e in v.log if e[0] == "servo"], [("servo", 9, 1900), ("servo", 9, 1100)])
        self.assertFalse(v.armed)
        self.assertTrue(w.stopped)
        # first servo: on the ground, disarmed
        self.assertFalse(v.servo_at[0][3])
        self.assertEqual(v.servo_at[0][2], 0.0)
        # second servo: in the air, within the arrival radius of the target
        lat, lon, alt, armed = v.servo_at[1]
        self.assertTrue(armed)
        self.assertLessEqual(haversine_m(lat, lon, 33.6820, 73.0470), 2.0)
        self.assertAlmostEqual(alt, 10, delta=1.0)

    def test_no_marker_returns_home_without_delivery(self):
        m, v, w = make(see_marker_after_s=None)
        self.assertEqual(run_mission(m), 1)
        self.assertNotIn("servo", [e[0] for e in v.log])
        self.assertEqual(v.log[-1], ("mode", "RTL"))

    def test_camera_failure_means_no_takeoff(self):
        m, v, w = make(fail_start=True)
        self.assertEqual(run_mission(m), 1)
        self.assertNotIn(("arm",), v.log)

    def test_camera_dies_mid_search_returns_home(self):
        m, v, w = make(see_marker_after_s=None, fail_after_s=5)
        self.assertEqual(run_mission(m), 1)
        self.assertEqual(v.log[-1], ("mode", "RTL"))

    def test_target_too_far_is_refused(self):
        m, v, w = make({"target": {"lat": 47.397742, "lon": 8.545594}}, see_marker_after_s=10)
        self.assertEqual(run_mission(m), 1)
        self.assertNotIn(("arm",), v.log)

    def test_low_battery_in_flight_returns_home(self):
        m, v, w = make(see_marker_after_s=None)
        def drain(t):
            if v.armed and t > 15:
                v.battery_pct = 20
        m.clock.on_tick.append(drain)
        self.assertEqual(run_mission(m), 1)
        self.assertEqual(v.log[-1], ("mode", "RTL"))

    def test_pilot_override_stops_commands(self):
        m, v, w = make(see_marker_after_s=None)
        def pilot(t):
            if v.mode == "AUTO" and t - v.auto_started > 5:
                v._mode = "LOITER"
        m.clock.on_tick.append(pilot)
        self.assertEqual(run_mission(m), 2)
        self.assertEqual([e for e in v.log if e[0] == "mode"][-1], ("mode", "AUTO"))

    def test_arm_refused_aborts_safely(self):
        m, v, w = make(see_marker_after_s=10)
        v.refuse_arm = True
        self.assertEqual(run_mission(m), 1)
        self.assertNotIn("takeoff", [e[0] for e in v.log])

    def test_lost_link_sends_nothing(self):
        m, v, w = make(see_marker_after_s=None)
        def cut(t):
            if v.mode == "AUTO":
                v.link = False
        m.clock.on_tick.append(cut)
        self.assertEqual(run_mission(m), 1)
        self.assertEqual([e for e in v.log if e[0] == "mode"][-1], ("mode", "AUTO"))


class CameraModelTests(unittest.TestCase):
    def test_project_and_back(self):
        import numpy as np
        from camera_model import CameraModel
        cam = CameraModel(640, 480, 62.2)
        for att in ((0, 0, 0), (0.1, -0.15, 1.2), (-0.05, 0.2, -2.5)):
            for n, e in ((0, 0), (3, -2), (-4, 5)):
                h = cam.ground_to_pixel_h(15, *att)
                p = h @ np.array([n, e, 1.0])
                u, v = p[0] / p[2], p[1] / p[2]
                back = cam.pixel_to_ground(u, v, 15, *att)
                self.assertAlmostEqual(back[0], n, places=6)
                self.assertAlmostEqual(back[1], e, places=6)

    def test_nose_is_image_top(self):
        from camera_model import CameraModel
        cam = CameraModel(640, 480, 62.2)
        n, e = cam.pixel_to_ground(320, 0, 10, 0, 0, 0)       # top centre, heading north
        self.assertGreater(n, 0)
        self.assertAlmostEqual(e, 0, places=6)
        n, e = cam.pixel_to_ground(320, 0, 10, 0, 0, 1.5708)  # heading east
        self.assertGreater(e, 0)


class ParserAndConfigTests(unittest.TestCase):
    def _write(self, text):
        f = tempfile.NamedTemporaryFile("w", suffix=".waypoints", delete=False)
        f.write(text)
        f.close()
        self.addCleanup(os.unlink, f.name)
        return f.name

    def test_parse_ok(self):
        items = parse_qgc_wpl(self._write(WPL))
        self.assertEqual(len(items), 4)
        self.assertEqual(items[1].frame, 3)
        self.assertAlmostEqual(items[2].y, 73.046)

    def test_parse_rejects_short_line(self):
        with self.assertRaises(ValueError):
            parse_qgc_wpl(self._write("QGC WPL 110\n0\t1\t0\t16\n"))

    def test_parse_rejects_bad_sequence(self):
        with self.assertRaises(ValueError):
            parse_qgc_wpl(self._write(WPL.replace("\n2\t", "\n5\t")))

    def test_config_validation(self):
        validate_config(BASE_CFG)
        bad = copy.deepcopy(BASE_CFG)
        bad["payload"]["pwm_at_target"] = 5000
        with self.assertRaises(ValueError):
            validate_config(bad)


if __name__ == "__main__":
    unittest.main()
