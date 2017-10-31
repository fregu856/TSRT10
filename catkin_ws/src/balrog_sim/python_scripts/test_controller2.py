#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

import tf
import numpy as np
import math

myGlobal_x = -1
myGlobal_y = 1

class Controller:
    def __init__(self):
        rospy.init_node("test_controller", anonymous=True)

        rospy.Subscriber("/estimated_pose", Float64MultiArray, self.pose_callback)

        rospy.Subscriber("/test_topic", Float64MultiArray, self.callback_function)

        self.control_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def callback_function(self, message_object):
        # get the actual message String that was published:
        received_message = message_object.data

        # print the received String:
        print "x-coordinate %f" % received_message[0]
        print "y-coordinate %f" % received_message[1]

        global myGlobal_x
        myGlobal_x = received_message[0]

        global myGlobal_y
        myGlobal_y = received_message[1]

    def pose_callback(self, msg_obj):
        pose = msg_obj.data

        self.x = pose[0]
        self.y = pose[1]
        self.theta = pose[2]

    def get_ctrl_output(self):
        # set a goal x, y, theta:
        x_g = myGlobal_x
        y_g = myGlobal_y
        th_g = 0.0

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
            print "True Robot angle: %f" % self.theta
            print "Robot angle: %f" % robot_angle
            print "Angle error: %f" % angle_error

            ANGULAR_VEL_THRESHOLD = np.pi/4
            LINEAR_VEL_THRESHOLD = 0.3

            if abs(angle_error) > 3*np.pi/180:
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
