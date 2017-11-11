#!/usr/bin/env python

import rospy

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

        rospy.Subscriber("/node_status", String, self.node_callback)

        #self.control_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10) # (simulation)
        self.control_pub = rospy.Publisher("/control_signals", Float64MultiArray, queue_size=10) # (physical robot)

        self.status_pub = rospy.Publisher("/controller_status", String, queue_size=10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.x_goal = 0
        self.y_goal = 0

        self.previous_linear_velocity = 0
        self.previous_angular_velocity = 0

        self.CONTROL_SIGNAL_MAX = 0.4
        self.CONTROL_SIGNAL_MIN = -self.CONTROL_SIGNAL_MAX

        self.target_position_not_published = True
        self.initial_angle_adjustment = True

        self.warning_flag = False

    def node_callback(self, msg_obj):
        self.warning_flag = True

    def goal_pos_callback(self, msg_obj):
        print "###################"
        print "goal_pos_callback"
        print "###################"

        goal_pos = msg_obj.data

        print "x_goal: %f" % goal_pos[0]
        print "y_goal: %f" % goal_pos[1]

        self.x_goal = goal_pos[0]
        self.y_goal = goal_pos[1]

        self.initial_angle_adjustment = True
        self.target_position_not_published = True

        self.warning_flag = False

    def est_pose_callback(self, msg_obj):
        pose = msg_obj.data

        self.x = pose[0]
        self.y = pose[1]
        self.theta = pose[2]

    def get_ctrl_output(self):
        x_g = self.x_goal
        y_g = self.y_goal

        x = self.x
        y = self.y
        theta = self.theta

        # CONTROLLER PARAMS:
        TOLERANCE = 0.07 # (how close to the target position the robot should stop [m])
        K_SPEED = 0.2 * 8
        K_ANGLE = 0.5 * 4 #0.573*5 (0.05 / deg2rad(5))
        # More aggresive angular adjustment when driving
        K_ANGLE_DURING_DRIVE = K_ANGLE * 2.5
        # More strict angle during initial_angle_adjustment
        if self.initial_angle_adjustment:
            ANGLE_ERROR_THRESHOLD = 2*np.pi/180
        else:
            ANGLE_ERROR_THRESHOLD = 6*np.pi/180

        WHEEL_BASE = 0.6138
        RADIUS_LEFT = 0.09
        RADIUS_RIGHT = RADIUS_LEFT
        SCALING_FACTOR = 0.1
        ANGULAR_VEL_THRESHOLD = np.pi/4
        LINEAR_VEL_THRESHOLD = 0.3
        CYCLES_TO_MAX_VELOCITY = 7
        ANGULAR_ACC_THRESHOLD = ANGULAR_VEL_THRESHOLD/CYCLES_TO_MAX_VELOCITY
        LINEAR_ACC_THRESHOLD = LINEAR_VEL_THRESHOLD/CYCLES_TO_MAX_VELOCITY

        print " "
        print "**********************************"
        print "Goal x: %f" % x_g
        print "Goal y: %f" % y_g
        print "Current x: %f" % x
        print "Current y: %f" % y

        distance_to_goal = np.sqrt((x - x_g)**2 + (y - y_g)**2)
        if distance_to_goal > TOLERANCE:
            goal_angle = np.arctan2(y_g - y, x_g - x)
            robot_angle = np.arctan2(np.sin(theta), np.cos(theta))
            angle_error_temp = robot_angle - goal_angle
            angle_error = np.arctan2(np.sin(angle_error_temp), np.cos(angle_error_temp))

            print "**********************************"
            print "Goal angle: %f" % goal_angle
            print "Robot angle: %f" % robot_angle
            print "**********************************"
            print "Angle error: %f" % angle_error

            # Regulator (Only P-part, velocity = P*error):
            if abs(angle_error) > ANGLE_ERROR_THRESHOLD:
                linear_velocity = 0

                if abs(-K_ANGLE*angle_error) > ANGULAR_VEL_THRESHOLD:
                    angular_velocity = -math.copysign(ANGULAR_VEL_THRESHOLD, angle_error)
                else:
                    angular_velocity = -K_ANGLE*angle_error
            else:
                self.initial_angle_adjustment = False
                if abs(-K_ANGLE_DURING_DRIVE*angle_error) > ANGULAR_VEL_THRESHOLD:
                    angular_velocity = -math.copysign(ANGULAR_VEL_THRESHOLD, angle_error)
                else:
                    angular_velocity = -K_ANGLE_DURING_DRIVE*angle_error

                if abs(K_SPEED*distance_to_goal) > LINEAR_VEL_THRESHOLD:
                    linear_velocity = LINEAR_VEL_THRESHOLD
                else:
                    linear_velocity = K_SPEED*distance_to_goal

            # Limit linear acceleration:
            if (linear_velocity - self.previous_linear_velocity) > LINEAR_ACC_THRESHOLD:
                linear_velocity = self.previous_linear_velocity + LINEAR_ACC_THRESHOLD
                print "linear acc limiter active"

            # Limit angular. Pos rotation when negative angle_error:
            if (angular_velocity - self.previous_angular_velocity)*np.sign(-angle_error) > ANGULAR_ACC_THRESHOLD:
                angular_velocity = self.previous_angular_velocity + math.copysign(ANGULAR_ACC_THRESHOLD, -angle_error)
                print "ang acc limiter active"



        else: # (if distance_to_goal <= TOLERANCE:)
            linear_velocity = 0
            angular_velocity = 0
            print "######## reached target position ########"
            #if self.target_position_not_published:
            msg = "reached target position"
            self.status_pub.publish(msg)
            self.target_position_not_published = False


        self.previous_linear_velocity = linear_velocity
        self.previous_angular_velocity = angular_velocity

        print "**********************************"
        print "Linear velocity %f" % linear_velocity
        print "Angular velocity %f" % angular_velocity


        # Turn linear_velocity and angular_velocity into velocity of each track
        angular_velocity_left = (2*linear_velocity - angular_velocity*WHEEL_BASE)/2/RADIUS_LEFT
        angular_velocity_right = (2*linear_velocity + angular_velocity*WHEEL_BASE)/2/RADIUS_RIGHT

        # Control signal is delta_angle of wheel. Scaling = 1/rospy.Rate()???
        control_signal_left = angular_velocity_left * SCALING_FACTOR
        control_signal_right = angular_velocity_right * SCALING_FACTOR

        print "**********************************"
        print "control_signal_left BEFORE saturation: %f" % control_signal_left
        print "control_signal_right BEFORE saturation: %f" % control_signal_right

        # To avoid too high control signals
        if control_signal_left > self.CONTROL_SIGNAL_MAX:
            control_signal_left = self.CONTROL_SIGNAL_MAX
        elif control_signal_left < self.CONTROL_SIGNAL_MIN:
            control_signal_left = self.CONTROL_SIGNAL_MIN
        if control_signal_right > self.CONTROL_SIGNAL_MAX:
            control_signal_right = self.CONTROL_SIGNAL_MAX
        elif control_signal_right < self.CONTROL_SIGNAL_MIN:
            control_signal_right = self.CONTROL_SIGNAL_MIN

        # Too low control signal will not turn wheels:
        MIN_CONTROL_SIGNAL = 0.14
        if control_signal_left > 0 and control_signal_left < MIN_CONTROL_SIGNAL:
            control_signal_left = MIN_CONTROL_SIGNAL
        if control_signal_left < 0 and control_signal_left > -MIN_CONTROL_SIGNAL:
            control_signal_left = -MIN_CONTROL_SIGNAL
        if control_signal_right > 0 and control_signal_right < MIN_CONTROL_SIGNAL:
            control_signal_right = MIN_CONTROL_SIGNAL
        if control_signal_right < 0 and control_signal_right > -MIN_CONTROL_SIGNAL:
            control_signal_right = -MIN_CONTROL_SIGNAL

        print "control_signal_left AFTER saturation: %f" % control_signal_left
        print "control_signal_right AFTER saturation: %f" % control_signal_right
        print "**********************************"

        # Publish control signal
        ctrl_output_msg = Float64MultiArray()
        ctrl_output_msg.data = [control_signal_left, control_signal_right]

        # ctrl_output_msg = Twist() # (simulation)
        # ctrl_output_msg.linear.x = linear_velocity
        # ctrl_output_msg.angular.z = angular_velocity

        return ctrl_output_msg

    def run(self):
        rate = rospy.Rate(10) # (10 Hz)

        while not rospy.is_shutdown():
            if not self.warning_flag:
                print "publishing control signal!"
                ctrl_output_msg = self.get_ctrl_output()
                self.control_pub.publish(ctrl_output_msg)
            else:
                print "[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]"
                print "[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]"
                print "[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]"
                print "[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]"
                print "stopped Balrog"
                print "[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]"
                print "[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]"
                print "[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]"
                print "[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]"
                ctrl_output_msg = Float64MultiArray()
                ctrl_output_msg.data = [0.0, 0.0]
                self.control_pub.publish(ctrl_output_msg)

            rate.sleep() # (to get it to loop with 10 Hz)

if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()
