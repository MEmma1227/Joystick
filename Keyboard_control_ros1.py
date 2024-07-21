import math
import time
import socket
import tty

# Import necessary modules for ROS 1 communication
import os  # Operating system module
import select  # I/O multiplexing module
import sys  # System-specific parameters and functions module
import rospy  # ROS 1 client library
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

speed = 0.4

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""
    return key

def keyboard_adjust_speed():
    global speed
    x = 0
    rz = 0
    key = get_key()

    if key == 'u':
        speed += 0.1
        print(speed)
    elif key == 'j':
        speed -= 0.1
        print(speed)

    if key == 'w': # 往前
        x = speed
        rz = 0
    elif key == 'x': # 往後
        x = -speed
        rz = 0
    elif key == 'a': # 原地往左轉
        x = 0
        rz = -speed / 2.0
    elif key == 'd': # 原地往右轉
        x = 0
        rz = speed / 2.0
    elif key == 's': # 原地不動
        x = 0
        rz = 0
    elif key == 'e': # 往右前
        x = speed
        rz = speed
    elif key == 'q': # 往左前
        x = speed
        rz = -speed
    elif key == 'c': # 往右後
        x = -speed
        rz = -speed
    elif key == 'z': # 往左後
        x = -speed
        rz = speed

    return x, rz

def main():
    rospy.init_node('keyboard_control')
    pub = rospy.Publisher('/amr_control_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        x, rz = keyboard_adjust_speed()
        twist = Twist()
        twist.linear.x = x / 1.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = rz / 1.0
        pub.publish(twist)

        key = get_key()
        if key == '\x03':
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            pub.publish(twist)
            break
        rate.sleep()

if __name__ == "__main__":
    main()
