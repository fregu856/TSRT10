#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

import tf

import numpy as np

pub = rospy.Publisher("/odom_pose", Float64MultiArray, queue_size=10)

r_w = 0.09 # (wheel radius)
b = 0.6138 # (wheel base?) 0.625

x = 0
y = 0
theta = 0

br = tf.TransformBroadcaster()

def wrapToPi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

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

    delta_theta = (delta_s_r - delta_s_l)/b

    delta_s = (delta_s_l + delta_s_r)/2

    x = x + delta_s*np.cos(theta + delta_theta/2)
    y = y + delta_s*np.sin(theta + delta_theta/2)
    theta = theta + delta_theta

    theta = wrapToPi(theta)

    state = [x, y, theta]
    #print state

    msg = Float64MultiArray()
    data = [x, y, theta]
    msg.data = data
    pub.publish(msg)

    pos = (x, y, 0)
    orient = tf.transformations.quaternion_from_euler(0, 0, theta)

    br.sendTransform(pos, orient, rospy.Time.now(), "base_footprint", "odom")

if __name__ == "__main__":
    # initialize this code as a ROS node named slam_odom:
    rospy.init_node("slam_odom", anonymous=True)

    # subscribe to the topic /encoder_data. That is, read every message (of type
    # Float64MultiArray) that is published on /encoder. We will do this by
    # calling the function callback_func every time a new message is published:
    rospy.Subscriber("/encoder_data", Float64MultiArray, callback_func)

    # keep python from exiting until this ROS node is stopped:
    rospy.spin()
