#!/usr/bin/env python3
"""Medical delivery mission over ROS1 + MAVROS.

    roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:115200
    python3 mission_ros1.py                      # uses mission_config.json
    python3 mission_ros1.py _config:=/path/my.json _mavros_ns:=/drone1/mavros

The MAVROS namespace comes from connection.mavros_ns in the config (or the
~mavros_ns parameter), so every topic and service uses the same prefix.
"""
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy  # noqa: E402
from mavros_msgs.msg import GlobalPositionTarget, State, Waypoint, WaypointReached  # noqa: E402
from mavros_msgs.srv import (  # noqa: E402
    CommandBool, CommandHome, CommandLong, CommandTOL, SetMode, WaypointClear, WaypointPush,
)
from geometry_msgs.msg import PoseStamped  # noqa: E402
from sensor_msgs.msg import BatteryState, NavSatFix  # noqa: E402
from std_msgs.msg import Float64, UInt32  # noqa: E402

from aruco_watcher import ArucoWatcher  # noqa: E402
from mission_common import Abort, load_config, log, parse_qgc_wpl  # noqa: E402
from mission_flow import DeliveryMission, run_mission  # noqa: E402

MAV_CMD_DO_SET_SERVO = 183
POSITION_ONLY = (GlobalPositionTarget.IGNORE_VX | GlobalPositionTarget.IGNORE_VY |
                 GlobalPositionTarget.IGNORE_VZ | GlobalPositionTarget.IGNORE_AFX |
                 GlobalPositionTarget.IGNORE_AFY | GlobalPositionTarget.IGNORE_AFZ |
                 GlobalPositionTarget.IGNORE_YAW | GlobalPositionTarget.IGNORE_YAW_RATE)


class RosVehicle:
    def __init__(self, ns, link_timeout_s):
        self.ns = ns.rstrip("/")
        self.link_timeout_s = link_timeout_s
        self._lock = threading.Lock()
        self._state = State()
        self._state_time = 0.0
        self._fix = None
        self._sats = None
        self._rel_alt = None
        self._battery = None
        self._orientation = None
        self._reached = set()

        t = self._topic
        rospy.Subscriber(t("state"), State, self._on_state)
        rospy.Subscriber(t("global_position/global"), NavSatFix, self._set("_fix"))
        rospy.Subscriber(t("global_position/raw/satellites"), UInt32, self._set("_sats", lambda m: m.data))
        rospy.Subscriber(t("global_position/rel_alt"), Float64, self._set("_rel_alt", lambda m: m.data))
        rospy.Subscriber(t("battery"), BatteryState, self._set("_battery"))
        rospy.Subscriber(t("local_position/pose"), PoseStamped,
                         self._set("_orientation", lambda m: m.pose.orientation))
        rospy.Subscriber(t("mission/reached"), WaypointReached, self._on_reached)
        self._setpoint_pub = rospy.Publisher(t("setpoint_raw/global"), GlobalPositionTarget, queue_size=1)

        self._srv = {}
        for name, srv_type in (("cmd/arming", CommandBool), ("set_mode", SetMode),
                               ("cmd/takeoff", CommandTOL), ("cmd/command", CommandLong),
                               ("cmd/set_home", CommandHome),
                               ("mission/push", WaypointPush), ("mission/clear", WaypointClear)):
            try:
                rospy.wait_for_service(t(name), timeout=30)
            except rospy.ROSException:
                raise Abort(f"MAVROS service {t(name)} not available; is MAVROS running "
                            f"with namespace {self.ns}?", "NONE")
            self._srv[name] = rospy.ServiceProxy(t(name), srv_type)

    def _topic(self, name):
        return f"{self.ns}/{name}"

    def _set(self, attr, convert=lambda m: m):
        def cb(msg):
            with self._lock:
                setattr(self, attr, convert(msg))
        return cb

    def _on_state(self, msg):
        with self._lock:
            self._state = msg
            self._state_time = time.time()

    def _on_reached(self, msg):
        with self._lock:
            self._reached.add(msg.wp_seq)

    def _call(self, name, **kwargs):
        try:
            return self._srv[name](**kwargs)
        except (rospy.ServiceException, rospy.ROSException) as e:
            log(f"Service {name} failed: {e}")
            return None

    # ------------------------------------------------------------ adapter API

    def link_ok(self):
        return self._state.connected and time.time() - self._state_time < self.link_timeout_s

    def shutdown_requested(self):
        return rospy.is_shutdown()

    @property
    def armed(self):
        return self._state.armed

    @property
    def mode(self):
        return self._state.mode or None

    def rel_alt(self):
        return self._rel_alt

    def position(self):
        fix = self._fix
        if fix is None or fix.status.status < 0 or (fix.latitude == 0 and fix.longitude == 0):
            return None
        return fix.latitude, fix.longitude

    def pose(self):
        """(lat, lon, rel_alt, roll, pitch, yaw) in NED/FRD for the camera, or None."""
        pos, alt, q = self.position(), self._rel_alt, self._orientation
        if pos is None or alt is None or q is None:
            return None
        x, y, z, w = q.x, q.y, q.z, q.w
        # MAVROS orientation is ENU/FLU; convert to NED/FRD angles.
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        yaw_enu = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return pos[0], pos[1], alt, roll, -pitch, math.pi / 2 - yaw_enu

    def gps_ok(self, min_sats):
        fix, sats = self._fix, self._sats
        if fix is None:
            return False, "no GPS data yet"
        has_fix = fix.status.status >= 0
        return (has_fix and sats is not None and sats >= min_sats,
                f"fix {'yes' if has_fix else 'no'}, {sats} satellites")

    def battery(self):
        b = self._battery
        if b is None:
            return None, None
        pct = b.percentage * 100 if b.percentage == b.percentage and b.percentage >= 0 else None
        volts = b.voltage if b.voltage == b.voltage and b.voltage > 0 else None
        return pct, volts

    def reached(self):
        with self._lock:
            return set(self._reached)

    def clear_reached(self):
        with self._lock:
            self._reached.clear()

    def set_mode(self, name, timeout=8):
        deadline = time.time() + timeout
        while time.time() < deadline:
            resp = self._call("set_mode", base_mode=0, custom_mode=name)
            if resp is not None and not resp.mode_sent:
                log(f"MAVROS could not send mode {name}")
            for _ in range(10):
                if self._state.mode == name:
                    return True
                time.sleep(0.1)
        return False

    def arm(self):
        resp = self._call("cmd/arming", value=True)
        if resp is None or not resp.success:
            return False
        deadline = time.time() + 5
        while time.time() < deadline:
            if self._state.armed:
                return True
            time.sleep(0.1)
        return False

    def takeoff(self, alt):
        resp = self._call("cmd/takeoff", min_pitch=0, yaw=0, latitude=0, longitude=0, altitude=alt)
        return resp is not None and resp.success

    def goto(self, lat, lon, alt):
        sp = GlobalPositionTarget()
        sp.header.stamp = rospy.Time.now()
        sp.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        sp.type_mask = POSITION_ONLY
        sp.latitude, sp.longitude, sp.altitude = lat, lon, alt
        self._setpoint_pub.publish(sp)

    def set_servo(self, channel, pwm):
        resp = self._call("cmd/command", broadcast=False, command=MAV_CMD_DO_SET_SERVO,
                          confirmation=0, param1=channel, param2=pwm,
                          param3=0, param4=0, param5=0, param6=0, param7=0)
        return resp is not None and resp.success

    def lock_home_here(self):
        # current_gps=True: use the current location. ArduPilot locks a commanded home.
        resp = self._call("cmd/set_home", current_gps=True, yaw=0, latitude=0, longitude=0, altitude=0)
        return resp is not None and resp.success

    def upload_mission(self, items):
        resp = self._call("mission/clear")
        if resp is None or not resp.success:
            log("Mission clear failed")
            return False
        waypoints = []
        for it in items:
            wp = Waypoint()
            wp.frame, wp.command = it.frame, it.command
            wp.is_current, wp.autocontinue = bool(it.current), bool(it.autocontinue)
            wp.param1, wp.param2, wp.param3, wp.param4 = it.p1, it.p2, it.p3, it.p4
            wp.x_lat, wp.y_long, wp.z_alt = it.x, it.y, it.z
            waypoints.append(wp)
        resp = self._call("mission/push", start_index=0, waypoints=waypoints)
        if resp is None or not resp.success or resp.wp_transfered != len(waypoints):
            log(f"Mission push failed: {resp}")
            return False
        return True


def main():
    # disable_signals: Ctrl+C reaches our handler while ROS still works, so we can command RTL.
    rospy.init_node("medical_delivery_mission", disable_signals=True)
    cfg = load_config(rospy.get_param("~config", None))
    ns = rospy.get_param("~mavros_ns", cfg["connection"]["mavros_ns"])
    items = parse_qgc_wpl(cfg["mission"]["waypoint_file"])

    try:
        vehicle = RosVehicle(ns, cfg["safety"]["link_timeout_s"])
    except Abort as e:
        log(f"ABORT: {e}")
        return 1
    log("Waiting for MAVROS to connect to the flight controller...")
    deadline = time.time() + 30
    while not vehicle.link_ok() and time.time() < deadline:
        time.sleep(0.2)
    if not vehicle.link_ok():
        log("ABORT: MAVROS is not connected to the flight controller")
        return 1

    mission = DeliveryMission(vehicle, ArucoWatcher(cfg["aruco"], pose_fn=vehicle.pose), cfg, items)
    try:
        return run_mission(mission)
    finally:
        rospy.signal_shutdown("mission finished")


if __name__ == "__main__":
    sys.exit(main())
