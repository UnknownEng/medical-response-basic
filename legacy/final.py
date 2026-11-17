#!/usr/bin/env python3
import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong
from mavros_msgs.msg import State, WaypointReached
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import threading

current_state = State()
current_alt = 0.0
wp_reached = -1
safety_pressed = False

def state_cb(msg):
    global current_state
    current_state = msg

def alt_cb(msg):
    global current_alt
    current_alt = msg.pose.position.z

def wp_cb(msg):
    global wp_reached
    wp_reached = msg.wp_seq

def safety_cb(msg):
    global safety_pressed
    safety_pressed = msg.data

def wait_mode(mode):
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if current_state.mode == mode:
            break
        r.sleep()

def wait_arm(state=True):
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if current_state.armed == state:
            break
        r.sleep()

def wait_alt(target):
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if current_alt >= target * 0.95:
            break
        r.sleep()

def wait_wp(n):
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if wp_reached == n:
            break
        r.sleep()

def wait_disarm():
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if not current_state.armed:
            break
        r.sleep()

def wait_safety():
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if safety_pressed:
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

def call_servo(channel, pwm):
    rospy.wait_for_service('/mavros/cmd/command')
    cmd = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
    cmd(command=183, param1=channel, param2=pwm)

def goto(lat, lon, alt):
    p = PoseStamped()
    p.header.stamp = rospy.Time.now()
    p.pose.position.x = lat
    p.pose.position.y = lon
    p.pose.position.z = alt
    pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
    r = rospy.Rate(20)
    c = 0
    while not rospy.is_shutdown():
        pub.publish(p)
        if abs(current_alt - alt) < 1.0:
            c += 1
        if c > 50:
            break
        r.sleep()

def run():
    rospy.init_node("full_drone_mission")
    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, alt_cb)
    rospy.Subscriber("/mavros/mission/reached", WaypointReached, wp_cb)
    rospy.Subscriber("/safety_switch", Bool, safety_cb)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    call_mode("GUIDED")
    wait_mode("GUIDED")

    call_arm(True)
    wait_arm(True)

    call_takeoff(15)
    wait_alt(15)

    call_mode("AUTO")
    wait_mode("AUTO")

    wait_wp(4)

    call_mode("LAND")
    wait_mode("LAND")
    wait_alt(0.3)
    wait_disarm()

    call_servo(6, 1900)

    wait_safety()

    call_mode("GUIDED")
    wait_mode("GUIDED")

    call_arm(True)
    wait_arm(True)

    call_takeoff(15)
    wait_alt(15)

    goto(47.397742, 8.545594, 15)

    call_servo(6, 1100)

    call_mode("RTL")
    wait_mode("RTL")

    wait_disarm()

if __name__ == "__main__":
    run()
