#!/usr/bin/env python

# This code runs at 10 Hz and computes the control signals (angular velocities
# for the left and right wheels) which are published on the topic /control_signals.
# The controller takes as input the current robot pose estimate (outputted by
# the SLAM node) and a goal position [x,y] (usually outputted by the coordinator).

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

import tf
import numpy as np
import math

class Controller:
    def __init__(self):
        # Initialize this code as a ROS node named "controller_node"
        rospy.init_node("controller_node", anonymous=True)

        # Get latest pose of the robot
        rospy.Subscriber("/estimated_pose", Float64MultiArray, self.est_pose_callback)

        # Get new goal position
        rospy.Subscriber("/goal_pos", Float64MultiArray, self.goal_pos_callback)

        # Get published warnings (when a warning is published: stop Balrog)
        rospy.Subscriber("/node_status", String, self.warning_callback)

        # To start Balrog, "start" has to be written on this topic
        rospy.Subscriber("/start_topic", String, self.start_callback)

        # Topic to change the linear speed of Balrog
        rospy.Subscriber("/change_velocity", String, self.velocitychange_callback)

        # Topic for control signals
        self.control_pub = rospy.Publisher("/control_signals", Float64MultiArray, queue_size=10) # (physical robot)
        #self.control_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10) # (simulation)

        # Topic to publish if the goal position is reached
        self.status_pub = rospy.Publisher("/controller_status", String, queue_size=10)

        # Current state of the robot. Updated in est_pose_callback
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Current goal position. Updated in goal_pos_callback
        self.x_goal = 0
        self.y_goal = 0

        # Store latest speed for acceleration reduction
        self.previous_linear_velocity = 0
        self.previous_angular_velocity = 0

        # Maximum control signal sent to the robot
        self.CONTROL_SIGNAL_MAX = 0.4
        self.CONTROL_SIGNAL_MIN = -self.CONTROL_SIGNAL_MAX

        # Global variables to change the behavior of the controller
        self.initial_angle_adjustment = True
        self.index = 0

        self.warning_flag = False

        self.start = False

        # Max velocity can be updated with the /change_velocity topic
        self.updated_max_velocity = 0

    # Callback function for the /start_topic topic
    def start_callback(self, msg_obj):
        msg_string = msg_obj.data

        if msg_string == "start":
            self.start = True

    # Callback function for the /goal_pos topic
    def goal_pos_callback(self, msg_obj):
        goal_pos = msg_obj.data

        print "##########################"
        print "New goal position received"
        print "x_goal: %f" % goal_pos[0]
        print "y_goal: %f" % goal_pos[1]
        print "##########################"

        self.x_goal = goal_pos[0]
        self.y_goal = goal_pos[1]

        self.initial_angle_adjustment = True

        self.warning_flag = False

    # Callback function for the /estimated_pose topic
    def est_pose_callback(self, msg_obj):
        pose = msg_obj.data

        self.x = pose[0]
        self.y = pose[1]
        self.theta = pose[2]

    # Callback function for the /node_status topic
    def warning_callback(self, msg_obj):
        self.warning_flag = True

    # Callback function for the /change_velocity topic
    def velocitychange_callback(self, msg_obj):
        self.updated_max_velocity = float(msg_obj.data)
        print "New velocity: %f" %self.updated_max_velocity

    # Function that computes he control signals
    def get_ctrl_output(self):
        x_g = self.x_goal
        y_g = self.y_goal

        x = self.x
        y = self.y
        theta = self.theta

        # CONTROLLER PARAMS:
        TOLERANCE = 0.07 # how close to the target position the robot should stop [m]
        K_SPEED = 0.1 * 8
        K_ANGLE = 0.5 * 4 #0.573*5 (0.05 / deg2rad(5))
        # More aggresive angular adjustment when driving
        K_ANGLE_DURING_DRIVE = K_ANGLE * 2.5

        # More strict angle error after new goal received (initial_angle_adjustment)
        if self.initial_angle_adjustment:
            ANGLE_ERROR_THRESHOLD = 2*np.pi/180
        else:
            ANGLE_ERROR_THRESHOLD = 6*np.pi/180

        # Vehicle paramters
        WHEEL_BASE = 0.6138
        RADIUS_LEFT = 0.09
        RADIUS_RIGHT = RADIUS_LEFT

        # Regulator paramters
        SCALING_FACTOR = 0.1 # Scaling from angular velocity of the wheel to control signal to RPi
        ANGULAR_VEL_THRESHOLD = np.pi/4

        # Linear velocity can be updated from a topic to override the inital value when starting the node
        LINEAR_VEL_THRESHOLD = 0.3
        if self.updated_max_velocity > 0:
            LINEAR_VEL_THRESHOLD = self.updated_max_velocity

        CYCLES_TO_MAX_VELOCITY = 7
        ANGULAR_ACC_THRESHOLD = ANGULAR_VEL_THRESHOLD/CYCLES_TO_MAX_VELOCITY
        LINEAR_ACC_THRESHOLD = LINEAR_VEL_THRESHOLD/CYCLES_TO_MAX_VELOCITY

        print " "
        print "******* Controller status ********"
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

            # Regulator (Only P-part, velocity = P*error)
            # If too large angle error -> turn around its axis
            if abs(angle_error) > ANGLE_ERROR_THRESHOLD:
                linear_velocity = 0

                # Angular velocity limiter
                if abs(-K_ANGLE*angle_error) > ANGULAR_VEL_THRESHOLD:
                    angular_velocity = -math.copysign(ANGULAR_VEL_THRESHOLD, angle_error)
                else:
                    angular_velocity = -K_ANGLE*angle_error

            # Move forward towards goal position
            else:
                self.initial_angle_adjustment = False
                # Angular velocity limiter
                if abs(-K_ANGLE_DURING_DRIVE*angle_error) > ANGULAR_VEL_THRESHOLD:
                    angular_velocity = -math.copysign(ANGULAR_VEL_THRESHOLD, angle_error)
                else:
                    angular_velocity = -K_ANGLE_DURING_DRIVE*angle_error

                # Linear velocity limiter
                if abs(K_SPEED*distance_to_goal) > LINEAR_VEL_THRESHOLD:
                    linear_velocity = LINEAR_VEL_THRESHOLD
                else:
                    linear_velocity = K_SPEED*distance_to_goal

            # Limit linear acceleration:
            if (linear_velocity - self.previous_linear_velocity) > LINEAR_ACC_THRESHOLD:
                linear_velocity = self.previous_linear_velocity + LINEAR_ACC_THRESHOLD
                print "linear acc limiter active"

            # Limit angular. Positive rotation when negative angle error:
            if (angular_velocity - self.previous_angular_velocity)*np.sign(-angle_error) > ANGULAR_ACC_THRESHOLD:
                angular_velocity = self.previous_angular_velocity + math.copysign(ANGULAR_ACC_THRESHOLD, -angle_error)
                print "ang acc limiter active"



        else: # (if distance_to_goal <= TOLERANCE:)
            # # Optional code to publish on topic not that often.
            # # When adding a manual route in the coordiantor, the first position
            # # is ignored probly due to the hign rate of publishing
            # if self.index < 1:
            #     msg = "reached target position"
            #     self.status_pub.publish(msg)
            #     self.index += 1
            # elif self.index > 5:
            #     self.index = 0

            linear_velocity = 0
            angular_velocity = 0
            print "**********************************"
            print "######## REACHED TARGET POSITION ########"


            msg = "reached target position"
            self.status_pub.publish(msg)


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
        # print "control_signal_left BEFORE saturation: %f" % control_signal_left
        # print "control_signal_right BEFORE saturation: %f" % control_signal_right

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

        print "control_signal_left after saturation: %f" % control_signal_left
        print "control_signal_right after saturation: %f" % control_signal_right
        print "**********************************"
        print ""

        # Publish control signal
        ctrl_output_msg = Float64MultiArray()
        ctrl_output_msg.data = [control_signal_left, control_signal_right]

        # (simulation:)
        # ctrl_output_msg = Twist()
        # ctrl_output_msg.linear.x = linear_velocity
        # ctrl_output_msg.angular.z = angular_velocity

        return ctrl_output_msg

    # Function that runs at 10 Hz, getting and publishing control signals:
    def run(self):
        # Specify the loop frequency:
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if not self.warning_flag and self.start:
                print "Control signal published"
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

            # Sleep to get the loop to run at 10 Hz:
            rate.sleep()

if __name__ == '__main__':
    # Create a Controller object (this will run its __init__ function):
    ctrl = Controller()

    ctrl.run()
