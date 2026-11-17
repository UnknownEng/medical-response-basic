#!/usr/bin/env python3
"""Logs the simulated flight (position, mode, armed, servo 9) to CSV.

    python3 recorder.py tcp:127.0.0.1:5762 output/track.csv
Connects on a second SITL port so it doesn't disturb the mission script.
"""
import csv
import sys
import time

from pymavlink import mavutil

url, out_path = sys.argv[1], sys.argv[2]
deadline = time.time() + 120
while True:
    try:
        conn = mavutil.mavlink_connection(url, source_system=250, retries=0)
        break
    except Exception:
        if time.time() > deadline:
            sys.exit("recorder: could not connect")
        time.sleep(1)

# Lock onto the autopilot's heartbeat. ArduPilot forwards other links' heartbeats
# (e.g. the mission script's), so the first heartbeat seen may not be the autopilot.
while True:
    hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=60)
    if hb is None:
        sys.exit("recorder: no heartbeat")
    if hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
        conn.target_system, conn.target_component = hb.get_srcSystem(), hb.get_srcComponent()
        break
conn.mav.request_data_stream_send(conn.target_system, conn.target_component,
                                  mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
mode, armed, servo9, att, speed, batt = "", 0, 0, None, 0.0, -1
with open(out_path, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "wall", "lat", "lon", "rel_alt", "mode", "armed", "servo9",
                "roll", "pitch", "yaw", "groundspeed", "battery_pct"])
    t0 = time.time()
    while True:
        msg = conn.recv_match(blocking=True, timeout=1)
        if msg is None or msg.get_srcSystem() != conn.target_system:
            continue
        mtype = msg.get_type()
        if mtype == "HEARTBEAT" and msg.get_srcComponent() == conn.target_component:
            mode = mavutil.mode_string_v10(msg)
            armed = int(bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
        elif mtype == "SERVO_OUTPUT_RAW":
            servo9 = getattr(msg, "servo9_raw", 0)
        elif mtype == "ATTITUDE":
            att = (msg.roll, msg.pitch, msg.yaw)
        elif mtype == "VFR_HUD":
            speed = msg.groundspeed
        elif mtype == "SYS_STATUS":
            batt = msg.battery_remaining
        elif mtype == "GLOBAL_POSITION_INT" and att is not None:
            now = time.time()
            w.writerow([f"{now - t0:.2f}", f"{now:.3f}", msg.lat / 1e7, msg.lon / 1e7,
                        msg.relative_alt / 1000.0, mode, armed, servo9,
                        f"{att[0]:.4f}", f"{att[1]:.4f}", f"{att[2]:.4f}",
                        f"{speed:.2f}", batt])
            f.flush()
