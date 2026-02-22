#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import State

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def run():
    rospy.init_node("test_connection")
    rospy.Subscriber("drone1/mavros/state", State, state_cb)
    rate = rospy.Rate(2)
    
    while not rospy.is_shutdown():
        print(f"Connected: {current_state.connected}, Mode: {current_state.mode}, Armed: {current_state.armed}")
        rate.sleep()

if __name__ == "__main__":
    run()
