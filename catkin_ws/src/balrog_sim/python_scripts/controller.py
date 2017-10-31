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

        self.control_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)

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
        # set a goal x, y, theta:
        x_g = self.x_goal
        y_g = self.y_goal

        # CONTROL_LAW
        TOLERANCE = 0.01 # (how close to the target position the robot should stop [m])

        # CONTROLLER:
        K_SPEED = 0.25*4
        K_ANGLE = 0.573*4 # (0.05 / deg2rad(5));
        WHEEL_BASE = 0.605
        RADIUS_LEFT = 0.09
        RADIUS_RIGHT = RADIUS_LEFT
        distanceToGoal = np.sqrt((self.x - x_g)**2 + (self.y - y_g)**2)

        if distanceToGoal > TOLERANCE:
            goal_angle = np.arctan2(y_g - self.y, x_g - self.x)
            robot_angle = np.arctan2(np.sin(self.theta), np.cos(self.theta))
            angle_error_temp = - goal_angle + robot_angle
            angle_error = np.arctan2(np.sin(angle_error_temp), np.cos(angle_error_temp))

            print " "
            print "Goal x: %f" % x_g
            print "Goal y: %f" % y_g
            print "Current x: %f" % self.x
            print "Current y: %f" % self.y
            print "Goal angle: %f" % goal_angle
            print "Robot angle: %f" % robot_angle
            print "Angle error: %f" % angle_error

            ANGULAR_VEL_THRESHOLD = np.pi/4
            LINEAR_VEL_THRESHOLD = 0.3

            if self.initial_angle_adjustment:
                if abs(angle_error) > 3*np.pi/180:
                    print "#####"
                    print "initial angle adjustment, angle error still too large"
                    print "#####"
                    linearVelocity = 0
                    if abs(-K_ANGLE*angle_error) > ANGULAR_VEL_THRESHOLD:
                        angularVelocity = -math.copysign(ANGULAR_VEL_THRESHOLD, angle_error)
                    else:
                        angularVelocity = -K_ANGLE*angle_error
                else:
                    self.initial_angle_adjustment = False
                    angularVelocity = -K_ANGLE*angle_error
                    if abs(K_SPEED*distanceToGoal) > LINEAR_VEL_THRESHOLD:
                        linearVelocity = LINEAR_VEL_THRESHOLD
                    else:
                        linearVelocity = K_SPEED*distanceToGoal
            else:
                if abs(angle_error) > 15*np.pi/180:
                    print "#####"
                    print "angle error too large!"
                    print "#####"
                    linearVelocity = 0
                    if abs(-K_ANGLE*angle_error) > ANGULAR_VEL_THRESHOLD:
                        angularVelocity = -math.copysign(ANGULAR_VEL_THRESHOLD, angle_error)
                    else:
                        angularVelocity = -K_ANGLE*angle_error
                else:
                    angularVelocity = -K_ANGLE*angle_error
                    if abs(K_SPEED*distanceToGoal) > LINEAR_VEL_THRESHOLD:
                        linearVelocity = LINEAR_VEL_THRESHOLD
                    else:
                        linearVelocity = K_SPEED*distanceToGoal

            print "linvel %f" % linearVelocity
            print "angvel %f" % angularVelocity
        else:
            linearVelocity = 0
            angularVelocity = 0
            print "reached target position"
            msg = "reached target position"
            self.status_pub.publish(msg)

        cmd = Twist()
        cmd.linear.x = linearVelocity
        cmd.angular.z = angularVelocity
        return cmd

    def run(self):
        rate = rospy.Rate(10) # (10 Hz)
        while not rospy.is_shutdown():
            ctrl_output = self.get_ctrl_output()
            self.control_pub.publish(ctrl_output)
            rate.sleep() # (to get it to loop with 10 Hz)

if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()
