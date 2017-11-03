#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

import tf
import numpy as np
import math

class Controller:
    def __init__(self):
        rospy.init_node("controller_node", anonymous=True)

        rospy.Subscriber("/estimated_pose", Float64MultiArray, self.est_pose_callback)

        rospy.Subscriber("/goal_pos", Float64MultiArray, self.goal_pos_callback)

        self.control_pub = rospy.Publisher("/control_signals", Float64MultiArray, queue_size=10)

        self.status_pub = rospy.Publisher("/controller_status", String, queue_size=10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.x_goal = 0
        self.y_goal = 0

        self.initial_angle_adjustment = True

    def goal_pos_callback(self, msg_obj):
        goal_pos = msg_obj.data

        print "x_goal: %f" % goal_pos[0]
        print "y_goal: %f" % goal_pos[1]

        self.x_goal = goal_pos[0]
        self.y_goal = goal_pos[1]

        self.initial_angle_adjustment = True

    def est_pose_callback(self, msg_obj):
        pose = msg_obj.data

        self.x = pose[0]
        self.y = pose[1]
        self.theta = pose[2]

    def get_ctrl_output(self):
        ctrl_msg = Float64MultiArray()
        ctrl_msg.data = [0.25, -0.25]

        return ctrl_msg

    def run(self):
        rate = rospy.Rate(1) # (10 Hz)
        while not rospy.is_shutdown():
            ctrl_msg = self.get_ctrl_output()
            self.control_pub.publish(ctrl_msg)
            rate.sleep() # (to get it to loop with 10 Hz)

if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()
