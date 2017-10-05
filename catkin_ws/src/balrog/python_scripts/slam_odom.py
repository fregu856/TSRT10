#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

import numpy as np

r_w = 0.09 # (wheel radius)
b = 0.62 # (wheel base?)

x = 0
y = 0
theta = 0

def callback_func(msg_obj):
    global x, y, theta

    data = msg_obj.data
    #print data
    odoRight = data[0]
    odoLeft = data[1]

    delta_theta_l = odoLeft
    delta_theta_r = odoRight

    delta_s_l = delta_theta_l*r_w
    delta_s_r = delta_theta_r*r_w

    delta_theta = (delta_s_l - delta_s_r)/b

    delta_s = (delta_s_l + delta_s_r)/2

    x = x + delta_s*np.cos(theta + delta_theta/2)
    y = y + delta_s*np.sin(theta + delta_theta/2)
    theta = theta + delta_theta

    state = [x, y, theta]
    print state

if __name__ == "__main__":
    # initialize this code as a ROS node named slam_odom:
    rospy.init_node("slam_odom", anonymous=True)

    # subscribe to the topic /encoder_data. That is, read every message (of type
    # Float64MultiArray) that is published on /encoder. We will do this by
    # calling the function callback_func every time a new message is published:
    rospy.Subscriber("/encoder_data", Float64MultiArray, callback_func)

    # # create a publisher that publishes messages of type String on the
    # # topic /test_topic:
    # pub = rospy.Publisher("/test_topic", String, queue_size=10)

    # keep python from exiting until this ROS node is stopped:
    rospy.spin()
