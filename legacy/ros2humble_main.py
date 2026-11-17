import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointPush, CommandLong
from mavros_msgs.msg import GlobalPositionTarget, State
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import threading
import time

class DroneMission(Node):
    def __init__(self):
        super().__init__('drone_mission')
        self.bridge = CvBridge()
        self.state = None
        self.target_lat = 33.6844
        self.target_lon = 73.0479
        self.target_alt = 10.0
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.img_cb, 10)
        self.pos_pub = self.create_publisher(GlobalPositionTarget, '/mavros/setpoint_position/global', 10)
        self.aruco_detected = False
        self.aruco_thread_active = False
        self.declare_parameter("waypoint_file", ".waypoints")

        self.cli_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.cli_wp = self.create_client(WaypointPush, '/mavros/mission/push')
        self.cli_takeoff = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.cli_long = self.create_client(CommandLong, '/mavros/cmd/command')

    def state_cb(self, msg):
        self.state = msg

    def img_cb(self, msg):
        if not self.aruco_thread_active:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
        if ids is not None:
            self.aruco_detected = True

    def call(self, client, req):
        while not client.wait_for_service(timeout_sec=1.0):
            pass
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        return self.call(self.cli_mode, req)

    def arm(self, val=True):
        req = CommandBool.Request()
        req.value = val
        return self.call(self.cli_arm, req)

    def takeoff(self, alt):
        req = CommandTOL.Request()
        req.altitude = alt
        return self.call(self.cli_takeoff, req)

    def send_servo(self, servo, pwm):
        req = CommandLong.Request()
        req.command = 183
        req.param1 = servo
        req.param2 = pwm
        return self.call(self.cli_long, req)

    def goto(self, lat, lon, alt):
        msg = GlobalPositionTarget()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        self.pos_pub.publish(msg)

    def aruco_thread(self):
        self.aruco_thread_active = True
        count = 0
        while rclpy.ok() and not self.aruco_detected:
            time.sleep(0.1)
        self.set_mode("LAND")
        time.sleep(8)
        self.send_servo(9, 2000)
        time.sleep(3)
        self.aruco_thread_active = False

    def run(self):
        self.set_mode("GUIDED")
        self.arm(True)
        self.takeoff(5)
        thread = threading.Thread(target=self.aruco_thread)
        thread.start()
        r = self.create_rate(10)
        while rclpy.ok() and not self.aruco_detected:
            r.sleep()
        thread.join()
        self.set_mode("GUIDED")
        self.arm(True)
        self.takeoff(self.target_alt)
        while rclpy.ok():
            self.goto(self.target_lat, self.target_lon, self.target_alt)
            time.sleep(1)
            break
        self.send_servo(9, 2000)
        time.sleep(5)
        self.set_mode("RTL")

def main(args=None):
    rclpy.init(args=args)
    node = DroneMission()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
