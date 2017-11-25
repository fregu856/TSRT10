#!/usr/bin/env python

# This code reads steering commands from the topic /steering_cmds, translates
# those commands into control signals in the form of a (linear) velocity and an
# angular velocity, and finally publishes these control signals on the topic
# cmd_vel_mux/input/navi (which is just a black box that translates these signals
# into actual robot movement).

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Manual_controller:
    def __init__(self):
        # initialize this code as a ROS node named manual_controller_node:
        rospy.init_node("manual_controller_node", anonymous=True)

        # create a publisher that publishes messages of type Twist on the
        # topic cmd_vel_mux/input/navi:
        self.pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist,
                    queue_size=10)

        # subscribe to the topic /steering_cmds (all published messages on
        # this topic are of type String), i.e., call the function cmd_callback
        # every time a new message is published:
        rospy.Subscriber("/steering_cmds", String, self.cmd_callback)

        # initialize the throttle direction ("Forward", "Backward", or
        # "No_throttle") and the steering direction ("Right", "Left", or
        # "No_steering"):
        self.throttle_direction = "No_throttle"
        self.steering_direction = "No_steering"

        # initialize the control signals:
        self.v = 0 # (linear velocity)
        self.omega = 0 # (angular velocity)

        # set upper/lower limits on the control signals:
        self.v_max = 0.65
        self.omega_max = 1.5

    # define the callback function for the /steering_cmds subscriber:
    def cmd_callback(self, msg_obj):
        # get the published steering command (a String):
        cmd = msg_obj.data

        if cmd in ["Forward", "Backward", "No_throttle"]:
            self.throttle_direction = cmd

        elif cmd in ["Right", "Left", "No_steering"]:
            self.steering_direction = cmd

    # define a member function which computes new values for self.v and
    # self.omega based on the current values of self.v, self.omega,
    # self.throttle_direction and self.steering_direction:
    def get_control_signals(self):
        # compute self.omega:
        if self.steering_direction == "Right":
            if self.omega < - 0.001: # (if currently rotating to the left:)
                self.omega = 0
            else: # (if currently rotating to the right or not at all:)
                self.omega = self.omega + 0.3
        elif self.steering_direction == "Left":
            if self.omega > 0.001: # (if currently rotating to the right:)
                self.omega = 0
            else: # (if currently rotating to the left or not at all:)
                self.omega = self.omega - 0.3
        elif self.steering_direction == "No_steering":
            self.omega = 0

        # limit self.omega to [-self.omega_max, self.omega_max]:
        if self.omega > self.omega_max:
            self.omega = self.omega_max
        elif self.omega < -self.omega_max:
            self.omega = -self.omega_max

        # compute self.v:
        if self.throttle_direction == "Forward":
            if self.v < -0.001:
                self.v = 0
            else:
                self.v = self.v + 0.05
        elif self.throttle_direction == "Backward":
            if self.v > 0.001:
                self.v = 0
            else:
                self.v = self.v - 0.05
        elif self.throttle_direction == "No_throttle":
            self.v = 0

        # limit self.v to [-self.v_max, self.v_max]:
        if self.v > self.v_max:
            self.v = self.v_max
        elif self.v < -self.v_max:
            self.v = -self.v_max

        # pack self.v and self.omega in the format expected by the robot:
        control_signals = Twist()
        control_signals.linear.x = self.v
        control_signals.angular.z = -self.omega

        return control_signals

    # define a member function which computes new control signals and sends
    # these to the robot with a frequency of 10 Hz:
    def run(self):
        # specify the desired loop frequency in Hz:
        rate = rospy.Rate(10)

        while not rospy.is_shutdown(): # (while the ROS node is still active:)
            # compute new control signals:
            control_signals = self.get_control_signals()

            # publish the control signals (on the specified topic, i.e., on
            # cmd_vel_mux/input/navi):
            self.pub.publish(control_signals)

            # sleep to get a loop frequency of 10 Hz:
            rate.sleep()

if __name__ == "__main__":
    # create an instance of the Manual_controller class (this will run
    # its __init__ function):
    manual_controller = Manual_controller()

    # run the member fuction run defined above:
    manual_controller.run()
