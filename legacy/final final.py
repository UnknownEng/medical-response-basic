#!/usr/bin/env python3
import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong
from mavros_msgs.msg import State, WaypointReached
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from mavros_msgs.msg import GlobalPositionTarget
import threading

current_state = State()
current_alt = 0.0
wp_reached = -1

# CALLBACKS
def state_cb(msg):
    global current_state
    current_state = msg
    # This updates the drone's current mode, armed status, and connection status

def alt_cb(msg):
    global current_alt
    current_alt = msg.pose.position.z
    # This updates the current altitude in local frame

def wp_cb(msg):
    global wp_reached
    wp_reached = msg.wp_seq
    # This updates which waypoint has been reached

# WAIT FUNCTIONS
def wait_mode(mode):
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if current_state.mode == mode:
            break
        r.sleep()
    # Wait until drone enters the desired mode

def wait_arm(state=True):
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if current_state.armed == state:
            break
        r.sleep()
    # Wait until drone is armed/disarmed

def wait_alt(target):
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if current_alt >= target * 0.95:
            break
        r.sleep()
    # Wait until drone reaches target altitude (~95%)

def wait_wp(n):
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if wp_reached == n:
            break
        r.sleep()
    # Wait until drone reaches waypoint 'n'

def wait_disarm():
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if not current_state.armed:
            break
        r.sleep()
    # Wait until drone disarms

# SERVICE CALLS
def call_mode(mode):
    rospy.wait_for_service('/drone1/mavros/set_mode')
    set_mode = rospy.ServiceProxy('/drone1/mavros/set_mode', SetMode)
    set_mode(custom_mode=mode)
    # Request the drone to switch to 'mode'

def call_arm(val):
    rospy.wait_for_service('/drone1/mavros/cmd/arming')
    arm_srv = rospy.ServiceProxy('/drone1/mavros/cmd/arming', CommandBool)
    arm_srv(value=val)
    # Arm/disarm the drone

def call_takeoff(alt):
    rospy.wait_for_service('/drone1/mavros/cmd/takeoff')
    takeoff = rospy.ServiceProxy('/drone1/mavros/cmd/takeoff', CommandTOL)
    takeoff(altitude=alt, latitude=0, longitude=0, min_pitch=0, yaw=0)
    # Takeoff to 'alt' meters (local Z used, lat/lon ignored)

def call_servo(channel, pwm):
    rospy.wait_for_service('/drone1/mavros/cmd/command')
    cmd = rospy.ServiceProxy('/drone1/mavros/cmd/command', CommandLong)
    cmd(command=183, param1=channel, param2=pwm)
    # Activate servo on 'channel' with PWM signal

def goto_gps(lat, lon, alt):
    pub = rospy.Publisher("/drone1/mavros/setpoint_raw/global", GlobalPositionTarget, queue_size=10)
    rospy.sleep(0.5)

    sp = GlobalPositionTarget()
    sp.header.frame_id = "map"
    sp.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
    sp.type_mask = 0  # Use all fields

    sp.latitude = lat
    sp.longitude = lon
    sp.altitude = alt

    rate = rospy.Rate(10)
    for i in range(100):  # send for 10 seconds
        sp.header.stamp = rospy.Time.now()
        pub.publish(sp)
        rate.sleep()
    # Fly to GPS location (lat, lon, alt) for ~10 seconds

# CONTINUOUS ARM
def continuous_arm():
    rospy.loginfo("=== Trying to ARM continuously... ===")

    while not rospy.is_shutdown():
        call_mode("GUIDED")  # Switch to GUIDED before arming
        rospy.sleep(0.5)

        call_arm(True)  # Try to arm
        rospy.sleep(1)

        if current_state.armed:
            rospy.loginfo("=== Successfully ARMED! ===")
            return True  # Stop once armed
        else:
            rospy.loginfo("Arm failed... retrying")
    return False

# MAIN MISSION
def run():
    rospy.init_node("full_drone_mission")
    # Subscribers to monitor drone state, altitude, and waypoints
    rospy.Subscriber("/drone1/mavros/state", State, state_cb)
    rospy.Subscriber("/drone1/mavros/local_position/pose", PoseStamped, alt_cb)
    rospy.Subscriber("/drone1/mavros/mission/reached", WaypointReached, wp_cb)

    # Wait until connected
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    # Drone connected to FCU

    # Step 1: GUIDED mode
    call_mode("GUIDED")
    wait_mode("GUIDED")
    # Drone now in GUIDED mode

    # Step 2: Arm
    call_arm(True)
    wait_arm(True)
    # Drone armed

    # Step 3: Takeoff to 10 m
    call_takeoff(10)
    wait_alt(10)
    # Drone climbed to ~10 m

    # Step 4: Switch to AUTO to follow mission
    call_mode("AUTO")
    wait_mode("AUTO")
    # Drone now following mission waypoints

    # Step 5: Wait until waypoint 4 reached
    wait_wp(4)
    # Drone reached waypoint 4

    # Step 6: Land
    call_mode("LAND")
    wait_mode("LAND")
    wait_alt(0.3)
    wait_disarm()
    # Drone landed and disarmed

    # Step 7: Activate servo
    call_servo(9, 1900)
    # Servo moved

    # Step 8: GUIDED mode again
    call_mode("GUIDED")
    wait_mode("GUIDED")
    # Drone in GUIDED

    # Step 9: Continuous arm
    continuous_arm()
    # Drone armed (kept retrying if failed)

    # Step 10: Takeoff again to 10 m
    call_takeoff(10)
    wait_alt(10)
    # Drone climbed to ~10 m

    # Step 11: Fly to GPS location
    goto_gps(33.6384204,72.9955874 , 15)
    # Drone moves to the given GPS point for 10 seconds

    # Step 12: Activate servo again
    call_servo(9, 1900)
    # Servo moved

    # Step 13: Return to launch (RTL)
    call_mode("RTL")
    wait_mode("RTL")
    # Drone starts RTL

    # Step 14: Wait for disarm at home
    wait_disarm()
    # Drone landed and disarmed, mission complete

if __name__ == "__main__":
    run()
