#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, Range
from geometry_msgs.msg import Twist


def lane_detection(image):
    # TODO: Implement lane detection algorithm
    
    pass


def object_detection(image):
    # TODO: Implement object/garbage detection algorithm

    pass


def sonar_callback(msg):
    # TODO: Implement sonar callback function

    pass


def image_callback(msg):
    # TODO: Implement image callback function

    pass


def odom_callback(msg):
    # TODO: Implement odom callback function

    pass


if __name__ == '__main__':
    rospy.init_node('hamo')
    img_sub = rospy.Subscriber('/image', Image, image_callback)
    sonar_sub = rospy.Subscriber('/scan', Range, sonar_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    perm_val = 100  # Need to calibrate

    while not rospy.is_shutdown():
        # Lane Detection
        offtrack, offtrack_status = lane_detection(img_sub)  # Offtrack number for P controller

        # Object/Garbage Detection
        dis, status_obj = object_detection(img_sub)  # Distance of object from camera

        if status_obj:
            print("Garbage")
        else:
            print("Object")

        motor_cmd = Twist()

        # P controller
        # TODO: Implement P controller algorithm and update motor_cmd accordingly

        cmd_vel_pub.publish(motor_cmd)

        # Garbage Removal Status (Once sonar_dis < perm_val we know its garbage )
        if sonar_sub.range < perm_val:
            print("Garbage Detected from sonar")

        rospy.sleep(0.1)
