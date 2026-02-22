#!/usr/bin/env python3
import rospy
from mavros_msgs.srv import CommandBool

def call_arm(val):
    rospy.wait_for_service('/mavros/cmd/arming')
    arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    arm_srv(value=val)

def run():
    rospy.init_node("test_arm")
    input("Press Enter to arm...")
    call_arm(True)
    print("Drone armed!")
    input("Press Enter to disarm...")
    call_arm(False)
    print("Drone disarmed!")

if __name__ == "__main__":
    run()
