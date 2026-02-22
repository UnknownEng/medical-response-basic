import time
import os
import cv2
import numpy as np
import threading
from pymavlink import mavutil
from picamera2 import Picamera2

target_lat = 33.6844
target_lon = 73.0479
target_alt = 10

WAYPOINT_FILE = os.path.expanduser("~/Desktop/Code/step.waypoints")
CONNECTION_STRING = "/dev/ttyACM0"

def parse_mission(file_path):
    mission = []
    with open(file_path, "r") as f:
        lines = f.readlines()
    if not lines or not lines[0].startswith("QGC WPL"):
        raise ValueError("Invalid waypoint file format or empty file.")
    for line in lines[1:]:
        parts = line.strip().split('\t')
        if len(parts) < 12:
            continue
        seq, current, frame, command = map(int, parts[:4])
        p1, p2, p3, p4, lat, lon, alt = map(float, parts[4:11])
        autocontinue = int(parts[11])
        mission.append((seq, frame, command, current, autocontinue, p1, p2, p3, p4, lat, lon, alt))
    return mission

def upload_mission(mav, mission):
    mav.mav.mission_clear_all_send(mav.target_system, mav.target_component)
    time.sleep(1)
    mav.mav.mission_count_send(mav.target_system, mav.target_component, len(mission))
    sent = 0
    while sent < len(mission):
        msg = mav.recv_match(type='MISSION_REQUEST', blocking=True, timeout=10)
        if not msg:
            raise TimeoutError("Did not receive MISSION_REQUEST in time.")
        seq = msg.seq
        if seq >= len(mission):
            raise IndexError(f"Requested seq {seq} exceeds mission size {len(mission)}")
        wp = mission[seq]
        (seq, frame, command, current, autocontinue,
         p1, p2, p3, p4, lat, lon, alt) = wp
        mav.mav.mission_item_send(
            mav.target_system,
            mav.target_component,
            seq,
            frame,
            command,
            current,
            autocontinue,
            p1, p2, p3, p4,
            lat, lon, alt
        )
        sent += 1
    ack = mav.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
    if not (ack and getattr(ack, 'type', None) == 0):
        raise RuntimeError(f"Mission ACK failed or denied: {ack}")

def set_mode(mav, mode_name, timeout=10):
    mapping = mav.mode_mapping()
    if mode_name not in mapping:
        raise ValueError(f"Unknown mode: {mode_name}")
    mode_id = mapping[mode_name]
    mav.mav.set_mode_send(mav.target_system, mav.target_component,
                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    start = time.time()
    while time.time() - start < timeout:
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and getattr(hb, 'custom_mode', None) == mode_id:
            return True
        time.sleep(0.2)
    raise RuntimeError(f"Failed to set mode to {mode_name} within timeout.")

def arm_vehicle(mav, timeout=15):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    start = time.time()
    while time.time() - start < timeout:
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
        time.sleep(0.2)
    raise RuntimeError("Arming timed out or failed.")

def disarm_vehicle(mav, timeout=15):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    start = time.time()
    while time.time() - start < timeout:
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
        time.sleep(0.2)
    raise RuntimeError("Disarming timed out or failed.")

def takeoff(mav, altitude):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    while True:
        msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg and (msg.relative_alt / 1000.0) >= altitude - 0.5:
            break
        time.sleep(0.5)

def go_to_location(mav, lat, lon, alt):
    mav.mav.set_position_target_global_int_send(
        0,
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        int(0b110111111000),
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

def run_camera_aruco(mav, stop_event, target_lat, target_lon, target_alt):
    picam2 = None
    try:
        picam2 = Picamera2()
        picam2.configure(picam2.create_preview_configuration(main={"format": "BGR888", "size": (640, 480)}))
        picam2.start()
    except:
        stop_event.set()
        return
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 23
    parameters.adaptiveThreshWinSizeStep = 10
    parameters.adaptiveThreshConstant = 7
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
    consec_required = 3
    consec_count = 0
    detection_triggered = False
    while not stop_event.is_set():
        frame = picam2.capture_array()
        if frame is None:
            time.sleep(0.05)
            continue
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None and len(ids) > 0:
            max_area = 0
            for i, corner in enumerate(corners):
                pts = corner[0].reshape(-1, 2)
                area = 0.5 * abs(np.dot(pts[:,0], np.roll(pts[:,1], 1)) - np.dot(pts[:,1], np.roll(pts[:,0], 1)))
                if area > max_area:
                    max_area = area
            area_fraction = max_area / float(w * h)
            side_estimate = np.sqrt(max_area) if max_area > 0 else 0
            if area_fraction >= 0.002 or side_estimate >= 50:
                consec_count += 1
            else:
                consec_count = 0
            time.sleep(0.05)
        else:
            consec_count = 0
            time.sleep(0.05)
        if consec_count >= consec_required and not detection_triggered:
            detection_triggered = True
            set_mode(mav, "LAND")
            while True:
                hb = mav.recv_match(type='HEARTBEAT', blocking=True)
                if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    break
                time.sleep(1)
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                183, 0, 1, 0, 0, 0, 0, 0, 0
            )
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 9, 2000, 0, 0, 0, 0, 0
            )
            while True:
                hb = mav.recv_match(type='HEARTBEAT', blocking=True)
                if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    break
                time.sleep(1)
            stop_event.set()
            break
    try:
        if picam2 is not None:
            picam2.stop()
    except:
        pass

def main():
    mav = mavutil.mavlink_connection(CONNECTION_STRING)
    mav.wait_heartbeat()
    while True:
        msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg and getattr(msg, 'lat', None) is not None:
            home_lat = msg.lat / 1e7
            home_lon = msg.lon / 1e7
            break
        time.sleep(2)
    mission = parse_mission(WAYPOINT_FILE)
    upload_mission(mav, mission)
    set_mode(mav, "GUIDED")
    arm_vehicle(mav)
    stop_event = threading.Event()
    aruco_thread = threading.Thread(target=run_camera_aruco, args=(mav, stop_event, target_lat, target_lon, target_alt))
    aruco_thread.start()
    takeoff(mav, 15)
    set_mode(mav, "AUTO")
    mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
    aruco_thread.join()
    set_mode(mav, "GUIDED")
    arm_vehicle(mav)
    takeoff(mav, target_alt)
    while True:
        msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0
            dist = np.sqrt((lat - target_lat)**2 + (lon - target_lon)**2)
            if dist < 0.00002:
                break
        go_to_location(mav, target_lat, target_lon, target_alt)
        time.sleep(1)
    mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 9, 2000, 0, 0, 0, 0, 0)
    time.sleep(5)
    set_mode(mav, "RTL")

if __name__ == "__main__":
    main()
