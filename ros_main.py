#!/usr/bin/env python3
import rospy
import threading
import numpy as np
import cv2
import os
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear
from mavros_msgs.msg import State, Waypoint, GlobalPositionTarget
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from picamera2 import Picamera2

class MavrosMissionNode:
    def __init__(self):
        rospy.init_node("mavros_mission_node")

        # Parameters
        self.wp_file = os.path.expanduser(rospy.get_param("~waypoint_file", "~/.waypoints"))
        self.target_lat = rospy.get_param("~target_lat", 33.6844)
        self.target_lon = rospy.get_param("~target_lon", 73.0479)
        self.target_alt = rospy.get_param("~target_alt", 10.0)
        self.takeoff_alt = rospy.get_param("~takeoff_alt", 5.0)

        # MAVROS topics/services
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")
        rospy.wait_for_service("/mavros/mission/push")
        rospy.wait_for_service("/mavros/mission/clear")
        self.arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.wp_push_srv = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
        self.wp_clear_srv = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)

        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_cb)

        self.current_state = State()
        self.current_gps = NavSatFix()

        self.stop_event = threading.Event()
        self.aruco_thread = threading.Thread(target=self.run_camera_aruco, daemon=True)

    def state_cb(self, msg):
        self.current_state = msg

    def gps_cb(self, msg):
        self.current_gps = msg

    def parse_waypoints(self, file_path):
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
            wp = Waypoint()
            wp.frame = frame
            wp.command = command
            wp.is_current = current
            wp.autocontinue = autocontinue
            wp.param1, wp.param2, wp.param3, wp.param4 = p1, p2, p3, p4
            wp.x_lat = lat
            wp.y_long = lon
            wp.z_alt = alt
            mission.append(wp)
        return mission

    def upload_mission(self):
        rospy.loginfo("Clearing mission...")
        self.wp_clear_srv()
        rospy.sleep(1)
        mission = self.parse_waypoints(self.wp_file)
        rospy.loginfo(f"Pushing {len(mission)} waypoints...")
        self.wp_push_srv(mission=mission)
        rospy.loginfo("Mission uploaded successfully")

    def arm_and_takeoff(self, alt):
        rospy.loginfo("Arming vehicle...")
        self.arming_srv(True)
        rospy.sleep(1)
        rospy.loginfo(f"Setting GUIDED mode...")
        self.set_mode_srv(custom_mode="GUIDED")
        rospy.sleep(1)
        rospy.loginfo(f"Takeoff to {alt}m")
        # Use MAVROS takeoff service
        rospy.wait_for_service("/mavros/cmd/takeoff")
        takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", Trigger)
        takeoff_srv()
        rospy.sleep(5)

    def run_mission_auto(self):
        rospy.loginfo("Switching to AUTO mode for mission")
        self.set_mode_srv(custom_mode="AUTO")
        rospy.sleep(2)

    def goto_target(self, lat, lon, alt):
        rospy.loginfo(f"Going to target: {lat}, {lon}, {alt}")
        pub = rospy.Publisher("/mavros/setpoint_position/global", GlobalPositionTarget, queue_size=10)
        msg = GlobalPositionTarget()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        rate = rospy.Rate(2)
        for _ in range(20):  # send multiple times to ensure reception
            pub.publish(msg)
            rate.sleep()

    def run_camera_aruco(self):
        rospy.loginfo("Starting ArUco detection thread")
        try:
            picam2 = Picamera2()
            picam2.configure(picam2.create_preview_configuration(main={"format": "BGR888", "size": (640, 480)}))
            picam2.start()
        except:
            rospy.logerr("Picamera2 failed to start")
            self.stop_event.set()
            return

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
        consec_required = 3
        consec_count = 0
        detected = False

        while not self.stop_event.is_set() and not rospy.is_shutdown():
            frame = picam2.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            if ids is not None and len(ids) > 0:
                consec_count += 1
            else:
                consec_count = 0
            rospy.sleep(0.05)

            if consec_count >= consec_required and not detected:
                rospy.loginfo("ArUco marker detected! Landing...")
                self.set_mode_srv(custom_mode="LAND")
                rospy.sleep(2)
                # Trigger servo (e.g. payload drop)
                pub = rospy.Publisher("/mavros/rc/override", PoseStamped, queue_size=1)
                # Add RC override here if needed
                detected = True
                self.stop_event.set()
                break

        picam2.stop()
        rospy.loginfo("ArUco thread finished")

    def run(self):
        self.upload_mission()
        self.arm_and_takeoff(self.takeoff_alt)
        self.aruco_thread.start()
        self.run_mission_auto()

        rospy.loginfo("Waiting for ArUco detection or mission completion...")
        while not rospy.is_shutdown() and not self.stop_event.is_set():
            rospy.sleep(1)

        rospy.loginfo("Mission node finished. Returning to RTL")
        self.set_mode_srv(custom_mode="RTL")
        rospy.sleep(2)


if __name__ == "__main__":
    node = MavrosMissionNode()
    node.run()