#!/usr/bin/env python3
import rospy
from mavros_msgs.srv import CommandLong

def call_servo(channel, pwm):
    rospy.wait_for_service('/mavros/cmd/command')
    cmd = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
    cmd(command=183, param1=channel, param2=pwm)

def run():
    rospy.init_node("test_servo")
    input("Press Enter to set servo to 1900 PWM...")
    call_servo(6, 1900)
    print("Servo set to 1900 PWM")
    input("Press Enter to set servo to 1100 PWM...")
    call_servo(6, 1100)
    print("Servo set to 1100 PWM")

if __name__ == "__main__":
    run()
