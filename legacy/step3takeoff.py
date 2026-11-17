#!/usr/bin/env python3
import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

current_state = State()
current_alt = 0.0

def state_cb(msg):
    global current_state
    current_state = msg

def alt_cb(msg):
    global current_alt
    current_alt = msg.pose.position.z

def wait_alt(target):
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        if current_alt >= target * 0.95:
            break
        r.sleep()

def call_mode(mode):
    rospy.wait_for_service('/mavros/set_mode')
    set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    set_mode(custom_mode=mode)

def call_arm(val):
    rospy.wait_for_service('/mavros/cmd/arming')
    arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    arm_srv(value=val)

def call_takeoff(alt):
    rospy.wait_for_service('/mavros/cmd/takeoff')
    takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    takeoff(altitude=alt, latitude=0, longitude=0, min_pitch=0, yaw=0)

def run():
    rospy.init_node("test_takeoff")
    rospy.Subscriber("/drone1/mavros/state", State, state_cb)
    rospy.Subscriber("/drone1/mavros/local_position/pose", PoseStamped, alt_cb)

    call_mode("GUIDED")
    call_arm(True)
    call_takeoff(3)
    wait_alt(3)
    print("Takeoff successful to 3 meters!")
    call_arm(False)
    print("Disarmed after landing test.")

if __name__ == "__main__":
    run()
