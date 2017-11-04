#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

import tf
import numpy as np

if __name__ == "__main__":
    # initialize this code as a ROS node named slam_pose:
    rospy.init_node("slam_pose", anonymous=True)

    pub = rospy.Publisher("/estimated_pose", Float64MultiArray, queue_size=10)

    listener = tf.TransformListener()

    # specify the desired loop frequency in Hz:
    rate = rospy.Rate(10)

    while not rospy.is_shutdown(): # (while the ROS node is still active:)
        try:
            (pos, orient) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            orient_euler = tf.transformations.euler_from_quaternion(orient)
            x = pos[0]
            y = pos[1]
            theta = orient_euler[2]
            pose = [x, y, theta]

            msg = Float64MultiArray()
            msg.data = pose
            #print msg
            pub.publish(msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Could not look up transform /map -> /base_footprint!"

        # sleep to get the desired loop frequency:
        rate.sleep()
