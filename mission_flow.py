"""The medical delivery mission, independent of how we talk to the drone.

mission_pymavlink.py and mission_ros1.py each provide a *vehicle adapter* and
hand it to DeliveryMission, so the mission steps and safety checks exist once.

A vehicle adapter must provide:
    link_ok() -> bool                 heartbeat from the flight controller is fresh
    shutdown_requested() -> bool
    armed -> bool, mode -> str|None   (properties)
    rel_alt() -> float|None           metres above home
    position() -> (lat, lon)|None
    gps_ok(min_sats) -> (bool, str)
    battery() -> (percent|None, volts|None)
    reached() -> set[int]             mission items reached since clear_reached()
    clear_reached()
    set_mode(name, timeout) -> bool   True once the flight controller reports the mode
    arm() -> bool                     True once armed
    takeoff(alt) -> bool              command accepted
    goto(lat, lon, alt)               position target, altitude relative to home
    set_servo(channel, pwm) -> bool
    upload_mission(items) -> bool
    lock_home_here() -> bool          set home to the current position (locked)
    pose() -> (lat, lon, rel_alt, roll, pitch, yaw)|None   used by the camera

Mission:
    preflight checks -> GUIDED, arm, take off -> AUTO route while watching for the
    ArUco marker -> LAND on it -> servo (pwm_at_marker), wait on the ground ->
    GUIDED, arm, take off -> fly to target -> servo (pwm_at_target) -> RTL.
"""
import time

from mission_common import (
    Abort, CameraError, MAV_CMD_NAV_LAND, MAV_CMD_NAV_RETURN_TO_LAUNCH,
    PilotOverride, haversine_m, log,
)


class DeliveryMission:
    def __init__(self, vehicle, watcher, cfg, items, clock=time.time, sleep=time.sleep, ask=input):
        self.v = vehicle
        self.watcher = watcher
        self.cfg = cfg
        self.items = items
        self.clock = clock
        self.sleep = sleep
        self.ask = ask
        self.phase = "preflight"
        self.allowed_modes = None  # None until we set the first mode
        self.started_at = None

    # ------------------------------------------------------------ safety

    def check(self):
        """Called in every wait loop. Raises Abort / PilotOverride."""
        v = self.v
        if v.shutdown_requested():
            raise Abort("shutdown requested", "RTL")
        if not v.link_ok():
            raise Abort("lost the link to the flight controller; ArduPilot's own failsafes must handle it", "NONE")
        if self.started_at is not None and self.clock() - self.started_at > self.cfg["mission"]["overall_timeout_s"]:
            raise Abort("overall mission time limit reached", "RTL")
        if v.armed:
            self._check_battery(in_flight=True)
        if self.allowed_modes is not None and v.mode is not None and v.mode not in self.allowed_modes:
            raise PilotOverride(
                f"flight mode changed to {v.mode} outside this script (RC pilot or ArduPilot failsafe)")
        if self.phase == "search" and self.watcher.failed.is_set():
            raise Abort("camera stopped working during the search", "RTL")

    def _check_battery(self, in_flight):
        s = self.cfg["safety"]
        pct, volts = self.v.battery()
        pct_min = s["min_battery_pct"] if in_flight else s["min_battery_pct_to_launch"]
        v_min = s["min_battery_v"] if in_flight else s["min_battery_v_to_launch"]
        if pct is not None and pct >= 0 and pct < pct_min:
            raise Abort(f"battery at {pct:.0f}%, below the {pct_min}% limit", "RTL")
        if volts is not None and v_min > 0 and 0 < volts < v_min:
            raise Abort(f"battery at {volts:.2f} V, below the {v_min} V limit", "RTL")

    def wait_until(self, condition, timeout, what, timeout_action="RTL"):
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            self.check()
            if condition():
                return
            self.sleep(0.2)
        raise Abort(f"timed out after {timeout:.0f} s waiting for {what}", timeout_action)

    def safe_action(self, action):
        """Best effort after an abort: put an armed drone into RTL or LAND."""
        if action not in ("RTL", "LAND"):
            return
        try:
            if not self.v.armed:
                return
            log(f"Commanding {action}.")
            if not self.v.set_mode(action, timeout=5):
                log(f"WARNING: flight controller did not confirm {action}. TAKE MANUAL CONTROL.")
        except Exception as e:
            log(f"WARNING: could not command {action} ({e}). TAKE MANUAL CONTROL.")

    # ------------------------------------------------------------ building blocks

    def change_mode(self, name):
        if self.allowed_modes is not None:
            self.allowed_modes = set(self.allowed_modes) | {name}
        log(f"Mode -> {name}")
        if not self.v.set_mode(name, timeout=8):
            raise Abort(f"flight controller did not switch to {name}", "RTL")
        self.allowed_modes = {name}

    def arm(self):
        attempts = self.cfg["safety"]["max_arm_attempts"]
        for attempt in range(1, attempts + 1):
            self.check()
            if self.v.arm():
                log("Armed.")
                return
            log(f"Arming attempt {attempt}/{attempts} refused (check the PreArm messages).")
            self.sleep(3)
        raise Abort("could not arm; fix the pre-arm errors and try again", "NONE")

    def takeoff(self, alt):
        log(f"Taking off to {alt} m.")
        if not self.v.takeoff(alt):
            raise Abort("takeoff command refused", "LAND")
        self.wait_until(lambda: (self.v.rel_alt() or 0.0) >= 0.95 * alt,
                        timeout=max(30.0, alt * 4), what=f"climb to {alt} m")

    def fly_to(self, lat, lon, alt, timeout, radius=None):
        radius = radius or self.cfg["target"]["arrival_radius_m"]
        log(f"Flying to {lat:.7f}, {lon:.7f} at {alt} m.")
        last_sent = [-1e9]

        def arrived():
            if self.clock() - last_sent[0] >= 1.0:  # re-send the target every second
                self.v.goto(lat, lon, alt)
                last_sent[0] = self.clock()
            pos, a = self.v.position(), self.v.rel_alt()
            if pos is None or a is None:
                return False
            return haversine_m(pos[0], pos[1], lat, lon) <= radius and abs(a - alt) <= 1.0

        self.wait_until(arrived, timeout, "arrival at the target")

    def wait_seconds(self, seconds, label=None):
        end = self.clock() + seconds
        next_log = self.clock()
        while self.clock() < end:
            self.check()
            if label and self.clock() >= next_log:
                log(f"{label} {end - self.clock():.0f} s")
                next_log = self.clock() + 5
            self.sleep(min(0.5, end - self.clock()))

    # ------------------------------------------------------------ mission phases

    def run(self):
        self.preflight()
        self.started_at = self.clock()
        self.flight_one()
        self.search()
        self.land_on_marker()
        self.ground_stop()
        self.deliver()
        log("Mission complete.")

    def preflight(self):
        c = self.cfg
        log("Waiting for GPS fix...")
        self.wait_until(lambda: self.v.gps_ok(c["safety"]["min_sats"])[0], 180, "a GPS fix", "NONE")
        log(f"GPS: {self.v.gps_ok(c['safety']['min_sats'])[1]}")
        self.wait_until(lambda: self.v.position() is not None, 30, "a position estimate", "NONE")
        self._check_battery(in_flight=False)

        home = self.v.position()
        t = c["target"]
        dist = haversine_m(home[0], home[1], t["lat"], t["lon"])
        if dist > t["max_distance_from_home_m"]:
            raise Abort(f"target is {dist:.0f} m from here (limit {t['max_distance_from_home_m']} m); "
                        "check the target coordinates in mission_config.json", "NONE")
        log(f"Home {home[0]:.7f}, {home[1]:.7f}; target is {dist:.0f} m away.")

        try:
            self.watcher.start()
        except CameraError as e:
            raise Abort(f"camera not working, not taking off: {e}", "NONE")

        log(f"Uploading {len(self.items)} mission items...")
        if not self.v.upload_mission(self.items):
            raise Abort("mission upload failed", "NONE")
        log("Mission uploaded.")

        # ArduPilot moves home to wherever it arms, so after re-arming at the marker
        # RTL would fly back to the marker. A home set by command is locked and
        # survives re-arming, so set it here at the launch point.
        # Refused until the EKF has a position estimate, so retry for a while.
        deadline = self.clock() + 120
        while not self.v.lock_home_here():
            if self.clock() > deadline:
                raise Abort("could not set and lock home at the launch point", "NONE")
            if deadline - self.clock() > 117:
                log("Waiting for the flight controller's position estimate to set home...")
            self.check()
            self.sleep(2)
        log("Home locked at the launch point.")

    def flight_one(self):
        self.phase = "takeoff"
        self.change_mode("GUIDED")
        self.arm()
        self.takeoff(self.cfg["mission"]["takeoff_alt_m"])

    def search(self):
        self.phase = "search"
        self.v.clear_reached()
        self.change_mode("AUTO")
        log("Flying the search route, watching for the marker.")
        min_alt = self.cfg["aruco"]["min_detect_alt_m"]
        last = self.items[-1]
        end_seqs = {last.seq}
        if last.command in (MAV_CMD_NAV_RETURN_TO_LAUNCH, MAV_CMD_NAV_LAND):
            end_seqs.add(last.seq - 1)

        def done():
            a = self.v.rel_alt()
            if a is not None and a >= min_alt:
                self.watcher.enabled.set()
            else:
                self.watcher.enabled.clear()
            return (self.watcher.detected.is_set()
                    or bool(end_seqs & self.v.reached())
                    or not self.v.armed)

        self.wait_until(done, self.cfg["mission"]["search_timeout_s"], "the ArUco marker")
        self.watcher.enabled.clear()
        if not self.watcher.detected.is_set():
            raise Abort("route finished without seeing the marker; no delivery", "RTL")

    def land_on_marker(self):
        hit = self.watcher.last
        where = f" at {hit['lat']:.7f}, {hit['lon']:.7f}" if hit.get("lat") is not None else ""
        log(f"Marker {hit['id']} confirmed{where} ({hit['frac']:.2%} of the frame).")
        if self.cfg["aruco"].get("center_before_landing") and hit.get("lat") is not None:
            self.center_over_marker(hit["lat"], hit["lon"])
        log("Landing.")
        self.phase = "land"
        self.change_mode("LAND")
        self.wait_until(lambda: not self.v.armed, self.cfg["mission"]["land_timeout_s"],
                        "landing and auto-disarm", timeout_action="NONE")
        log("Landed and disarmed.")

    def marker_fix(self, seconds):
        """Watch for `seconds` and return the median marker position, or None."""
        self.watcher.clear_hits()
        self.watcher.enabled.set()
        try:
            self.wait_seconds(seconds)
        finally:
            self.watcher.enabled.clear()
        fixes = [(h["lat"], h["lon"]) for h in self.watcher.hits() if h.get("lat") is not None]
        if not fixes:
            return None
        lats, lons = sorted(f[0] for f in fixes), sorted(f[1] for f in fixes)
        return lats[len(lats) // 2], lons[len(lons) // 2]

    def center_over_marker(self, lat, lon):
        """Hover over the marker's estimated position, re-measure from above, repeat."""
        a = self.cfg["aruco"]
        self.phase = "centering"
        self.change_mode("GUIDED")
        alt = a.get("center_alt_m", 8)
        for attempt in range(1, 4):
            self.fly_to(lat, lon, alt, 60, radius=0.4)
            self.wait_seconds(1.5)  # let it settle so the camera is level
            fix = self.marker_fix(2.0)
            if fix is None:
                log("Marker not visible from above; landing at the last estimate.")
                return
            moved = haversine_m(lat, lon, fix[0], fix[1])
            lat, lon = fix
            log(f"Centering pass {attempt}: marker estimate moved {moved:.2f} m.")
            if moved < 0.3:
                break
        self.fly_to(lat, lon, alt, 30, radius=0.3)

    def ground_stop(self):
        self.phase = "ground"
        p, g = self.cfg["payload"], self.cfg["ground"]
        log(f"Servo {p['servo_channel']} -> {p['pwm_at_marker']}")
        if not self.v.set_servo(p["servo_channel"], p["pwm_at_marker"]):
            raise Abort("servo command refused at the marker", "NONE")
        if g["confirm_with_enter"]:
            self.ask("Payload ready and people clear? Press Enter to take off again (Ctrl+C stops)... ")
        else:
            self.wait_seconds(g["wait_s"], "Taking off again in")
        self.check()
        self._check_battery(in_flight=False)

    def deliver(self):
        self.phase = "delivery"
        t, p = self.cfg["target"], self.cfg["payload"]
        self.change_mode("GUIDED")
        self.arm()
        self.takeoff(t["alt_m"])
        self.fly_to(t["lat"], t["lon"], t["alt_m"], t["goto_timeout_s"])
        if abs(t["drop_alt_m"] - t["alt_m"]) > 0.5:
            self.fly_to(t["lat"], t["lon"], t["drop_alt_m"], 60)
        log(f"Over the target. Servo {p['servo_channel']} -> {p['pwm_at_target']}")
        if not self.v.set_servo(p["servo_channel"], p["pwm_at_target"]):
            log("WARNING: servo command at the target was refused.")
        self.wait_seconds(p["release_wait_s"])

        self.phase = "return"
        self.change_mode("RTL")
        self.wait_until(lambda: not self.v.armed, self.cfg["mission"]["rtl_timeout_s"],
                        "return home and landing", timeout_action="NONE")


def run_mission(mission):
    """Run the mission and handle every way it can end. Returns an exit code."""
    try:
        mission.run()
        return 0
    except PilotOverride as e:
        log(f"STOPPING: {e}. No further commands will be sent.")
        return 2
    except Abort as e:
        log(f"ABORT: {e}")
        mission.safe_action(e.action)
        return 1
    except KeyboardInterrupt:
        log("Interrupted by user.")
        mission.safe_action("RTL")
        return 130
    except Exception as e:
        import traceback
        traceback.print_exc()
        log(f"ABORT: unexpected error: {e}")
        mission.safe_action("RTL")
        return 1
    finally:
        mission.watcher.stop()
