#!/usr/bin/env python3
"""Medical delivery mission over pymavlink (Raspberry Pi wired to a Pixhawk).

    python3 mission_pymavlink.py                       # uses mission_config.json
    python3 mission_pymavlink.py --config my.json
    python3 mission_pymavlink.py --url udp:127.0.0.1:14550 --camera 0   # SITL + webcam

Only one thread (the reader) ever receives from the MAVLink connection, and
every send goes through one lock, so the camera thread can't steal messages.
"""
import argparse
import queue
import sys
import threading
import time

from pymavlink import mavutil

from aruco_watcher import ArucoWatcher
from mission_common import Abort, GLOBAL_FRAMES, load_config, log, parse_qgc_wpl
from mission_flow import DeliveryMission, run_mission

mavlink = mavutil.mavlink
ARMED_FLAG = mavlink.MAV_MODE_FLAG_SAFETY_ARMED
# Use position only; ignore velocity, acceleration, yaw and yaw rate.
POSITION_ONLY = 0b110111111000
LOCAL_FRAMES = {1, 7, 8, 9}  # MAV_FRAME_LOCAL_NED / LOCAL_OFFSET_NED / BODY_NED / BODY_OFFSET_NED


def _result_name(result):
    try:
        return mavlink.enums["MAV_RESULT"][result].name
    except KeyError:
        return str(result)


class PymavlinkVehicle:
    def __init__(self, url, baud, link_timeout_s):
        self.conn = mavutil.mavlink_connection(
            url, baud=baud, source_system=255,
            source_component=mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
        self.link_timeout_s = link_timeout_s
        self.sysid = self.compid = None
        self.mav_type = None
        self._armed = False
        self._mode = None
        self._last_heartbeat = 0.0
        self._latest = {}
        self._reached = set()
        self._acks = queue.Queue()
        self._mission_msgs = queue.Queue()
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._stop = threading.Event()
        self._reader = None

    # ------------------------------------------------------------ connection

    def connect(self, timeout=30):
        """Wait for the autopilot's heartbeat (ignoring GCS heartbeats)."""
        log("Waiting for flight controller heartbeat...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            hb = self.conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb and hb.autopilot != mavlink.MAV_AUTOPILOT_INVALID:
                self.sysid, self.compid = hb.get_srcSystem(), hb.get_srcComponent()
                self.conn.target_system, self.conn.target_component = self.sysid, self.compid
                self._apply_heartbeat(hb)
                break
        else:
            raise Abort("no heartbeat from the flight controller", "NONE")
        log(f"Connected to system {self.sysid}, mode {self._mode}.")
        # 10 Hz so the camera's position estimates use fresh position/attitude.
        self._send("request_data_stream_send", self.sysid, self.compid,
                   mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        self._reader = threading.Thread(target=self._read_loop, name="mavlink", daemon=True)
        self._reader.start()

    def close(self):
        self._stop.set()
        if self._reader:
            self._reader.join(timeout=2)
        self.conn.close()

    def _send(self, name, *args):
        with self._send_lock:
            getattr(self.conn.mav, name)(*args)

    def _apply_heartbeat(self, hb):
        with self._lock:
            self.mav_type = hb.type
            self._armed = bool(hb.base_mode & ARMED_FLAG)
            self._mode = mavutil.mode_string_v10(hb)
            self._last_heartbeat = time.time()

    def _read_loop(self):
        last_own_hb = 0.0
        while not self._stop.is_set():
            # Our own heartbeat: if this script dies, ArduPilot's GCS failsafe
            # (FS_GCS_ENABLE with SYSID_MYGCS=255) can notice.
            if time.time() - last_own_hb >= 1.0:
                self._send("heartbeat_send", mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                           mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_own_hb = time.time()
            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.5)
            except Exception as e:
                log(f"MAVLink read error: {e}")
                time.sleep(0.5)
                continue
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype == "BAD_DATA" or msg.get_srcSystem() != self.sysid:
                continue
            if mtype == "STATUSTEXT":
                log(f"FC: {msg.text}")
            elif mtype == "HEARTBEAT":
                if msg.get_srcComponent() == self.compid:
                    self._apply_heartbeat(msg)
                continue
            elif mtype == "COMMAND_ACK":
                self._acks.put(msg)
            elif mtype in ("MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"):
                self._mission_msgs.put(msg)
            elif mtype == "MISSION_ITEM_REACHED":
                with self._lock:
                    self._reached.add(msg.seq)
            with self._lock:
                self._latest[mtype] = msg

    def _get(self, mtype):
        with self._lock:
            return self._latest.get(mtype)

    @staticmethod
    def _drain(q):
        while True:
            try:
                q.get_nowait()
            except queue.Empty:
                return

    # ------------------------------------------------------------ adapter API

    def link_ok(self):
        return time.time() - self._last_heartbeat < self.link_timeout_s

    def shutdown_requested(self):
        return False

    @property
    def armed(self):
        return self._armed

    @property
    def mode(self):
        return self._mode

    def rel_alt(self):
        m = self._get("GLOBAL_POSITION_INT")
        return None if m is None else m.relative_alt / 1000.0

    def position(self):
        m = self._get("GLOBAL_POSITION_INT")
        if m is None or (m.lat == 0 and m.lon == 0):
            return None
        return m.lat / 1e7, m.lon / 1e7

    def pose(self):
        """(lat, lon, rel_alt, roll, pitch, yaw) for the camera, or None."""
        pos, att = self._get("GLOBAL_POSITION_INT"), self._get("ATTITUDE")
        if pos is None or att is None or (pos.lat == 0 and pos.lon == 0):
            return None
        return (pos.lat / 1e7, pos.lon / 1e7, pos.relative_alt / 1000.0,
                att.roll, att.pitch, att.yaw)

    def gps_ok(self, min_sats):
        m = self._get("GPS_RAW_INT")
        if m is None:
            return False, "no GPS data yet"
        return (m.fix_type >= 3 and m.satellites_visible >= min_sats,
                f"fix type {m.fix_type}, {m.satellites_visible} satellites")

    def battery(self):
        m = self._get("SYS_STATUS")
        if m is None:
            return None, None
        pct = m.battery_remaining if m.battery_remaining >= 0 else None
        volts = m.voltage_battery / 1000.0 if m.voltage_battery not in (0, 65535) else None
        return pct, volts

    def reached(self):
        with self._lock:
            return set(self._reached)

    def clear_reached(self):
        with self._lock:
            self._reached.clear()

    def command(self, cmd, *params, timeout=3.0, retries=2):
        params = (list(params) + [0] * 7)[:7]
        for attempt in range(retries + 1):
            self._drain(self._acks)
            self._send("command_long_send", self.sysid, self.compid, cmd, attempt, *params)
            deadline = time.time() + timeout
            while time.time() < deadline:
                try:
                    ack = self._acks.get(timeout=0.2)
                except queue.Empty:
                    continue
                if ack.command != cmd:
                    continue
                if ack.result == mavlink.MAV_RESULT_ACCEPTED:
                    return True
                if ack.result == mavlink.MAV_RESULT_IN_PROGRESS:
                    continue
                log(f"Command {cmd} rejected: {_result_name(ack.result)}")
                return False
        log(f"Command {cmd}: no answer from the flight controller")
        return False

    def set_mode(self, name, timeout=8):
        mapping = mavutil.mode_mapping_byname(self.mav_type) or {}
        if name not in mapping:
            log(f"Mode {name} not available on this vehicle")
            return False
        deadline = time.time() + timeout
        while time.time() < deadline:
            # SET_MODE has exactly three fields: target_system, base_mode, custom_mode.
            self._send("set_mode_send", self.sysid,
                       mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping[name])
            for _ in range(10):
                if self._mode == name:
                    return True
                time.sleep(0.1)
        return False

    def arm(self):
        if not self.command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, retries=0, timeout=5):
            return False
        deadline = time.time() + 5
        while time.time() < deadline:
            if self._armed:
                return True
            time.sleep(0.1)
        return False

    def takeoff(self, alt):
        return self.command(mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt)

    def goto(self, lat, lon, alt):
        self._send("set_position_target_global_int_send", 0, self.sysid, self.compid,
                   mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, POSITION_ONLY,
                   int(lat * 1e7), int(lon * 1e7), float(alt),
                   0, 0, 0, 0, 0, 0, 0, 0)

    def set_servo(self, channel, pwm):
        return self.command(mavlink.MAV_CMD_DO_SET_SERVO, channel, pwm)

    def lock_home_here(self):
        # param1 = 1: use the current location. ArduPilot locks a commanded home.
        return self.command(mavlink.MAV_CMD_DO_SET_HOME, 1, retries=0)

    def upload_mission(self, items):
        self._drain(self._mission_msgs)
        self._send("mission_clear_all_send", self.sysid, self.compid)
        try:
            self._mission_msgs.get(timeout=3)  # clear's MISSION_ACK
        except queue.Empty:
            pass
        self._drain(self._mission_msgs)
        self._send("mission_count_send", self.sysid, self.compid, len(items))
        sent = set()
        while True:
            try:
                msg = self._mission_msgs.get(timeout=5)
            except queue.Empty:
                log("Mission upload: flight controller stopped asking for items")
                return False
            mtype = msg.get_type()
            if mtype == "MISSION_ACK":
                if msg.type == mavlink.MAV_MISSION_ACCEPTED and len(sent) == len(items):
                    return True
                log(f"Mission upload rejected (code {msg.type}, sent {len(sent)}/{len(items)})")
                return False
            if msg.seq >= len(items):
                log(f"Mission upload: asked for item {msg.seq} of {len(items)}")
                return False
            it = items[msg.seq]
            # ArduPilot wants MISSION_ITEM_INT whichever request type it sends.
            # x/y scaling per MAVLink: lat/lon * 1e7, local metres * 1e4, otherwise raw.
            if it.frame in GLOBAL_FRAMES:
                scale = 1e7
            elif it.frame in LOCAL_FRAMES:
                scale = 1e4
            else:
                scale = 1
            self._send("mission_item_int_send", self.sysid, self.compid, it.seq, it.frame,
                       it.command, it.current, it.autocontinue, it.p1, it.p2, it.p3, it.p4,
                       int(round(it.x * scale)), int(round(it.y * scale)), it.z)
            sent.add(msg.seq)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", help="path to mission_config.json")
    ap.add_argument("--url", help="override connection.pymavlink_url")
    ap.add_argument("--camera", help="override aruco.source ('picamera2' or a webcam number)")
    args = ap.parse_args()

    cfg = load_config(args.config)
    if args.url:
        cfg["connection"]["pymavlink_url"] = args.url
    if args.camera:
        cfg["aruco"]["source"] = args.camera
    items = parse_qgc_wpl(cfg["mission"]["waypoint_file"])

    vehicle = PymavlinkVehicle(cfg["connection"]["pymavlink_url"], cfg["connection"]["baud"],
                               cfg["safety"]["link_timeout_s"])
    try:
        vehicle.connect()
    except Abort as e:
        log(f"ABORT: {e}")
        return 1
    mission = DeliveryMission(vehicle, ArucoWatcher(cfg["aruco"], pose_fn=vehicle.pose), cfg, items)
    try:
        return run_mission(mission)
    finally:
        vehicle.close()


if __name__ == "__main__":
    sys.exit(main())
