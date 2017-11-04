#!/usr/bin/env python

# this code reads steering commands from the topic /steering_cmds, translates
# those commands into control signals in the form of a (linear) velocity and an
# angular velocity, and finally publishes these control signals on the topic
# cmd_vel_mux/input/navi (which is just a black box that translates these signals
# into actual robot movement).

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

def linear_interp(x_val, x_min, x_max, y_min, y_max):
    x_range = x_max - x_min
    y_range = y_max - y_min
    xy_ratio = float(x_range)/float(y_range)

    y = (x_val-x_min)/xy_ratio + y_min

    return y

class Manual_controller:
    def __init__(self):
        # initialize this code as a ROS node named manual_controller_node:
        rospy.init_node("manual_controller_node", anonymous=True)

        # create a publisher that publishes messages of type Twist on the
        # topic cmd_vel_mux/input/navi:
        self.pub = rospy.Publisher("/control_signals", Float64MultiArray, queue_size=10)

        # subscribe to the topic /steering_cmds (all published messages on
        # this topic are of type String), i.e., call the function cmd_callback
        # every time a new message is published:
        rospy.Subscriber("/steering_cmds", String, self.cmd_callback)

        # initialize the direction ("Forward", "Backward", "Right", "Left" or "No_directon"):
        self.direction = "No_direction"

        # initialize the control signals:
        self.omega_l = 0
        self.omega_r = 0

        # set upper/lower limits on the control signals:
        self.omega_max = 0.3

    # define the callback function for the /steering_cmds subscriber:
    def cmd_callback(self, msg_obj):
        # get the published steering command (a String):
        cmd = msg_obj.data

        if cmd in ["Forward", "Backward", "Right", "Left", "No_direction"]:
            self.direction = cmd

    # define a member function which computes new values for self.v and
    # self.omega based on the current values of self.v, self.omega,
    # self.throttle_direction and self.steering_direction:
    def get_control_signals(self):
        if self.direction == "Forward":
            self.omega_l += 0.05
            self.omega_r += 0.05

        elif self.direction == "Backward":
            self.omega_l -= 0.05
            self.omega_r -= 0.05

        elif self.direction == "Right":
            self.omega_l += 0.05
            self.omega_r -= 0.05

        elif self.direction == "Left":
            self.omega_l -= 0.05
            self.omega_r += 0.05

        elif self.direction == "No_direction":
            if self.omega_l > 0.01 and self.omega_r > 0.01:
                if self.omega_l < 0.1 and self.omega_r < 0.1:
                    self.omega_l = 0
                    self.omega_r = 0
                else:
                    self.omega_l -= 0.1
                    self.omega_r -= 0.1
            elif self.omega_l < -0.01 and self.omega_r < -0.01:
                if self.omega_l > -0.1 and self.omega_r > -0.1:
                    self.omega_l = 0
                    self.omega_r = 0
                else:
                    self.omega_l += 0.1
                    self.omega_r += 0.1
            elif self.omega_l < -0.01 and self.omega_r > 0.01:
                if self.omega_l > -0.15 and self.omega_r < 0.15:
                    self.omega_l = 0
                    self.omega_r = 0
                else:
                    self.omega_l += 0.15
                    self.omega_r -= 0.15
            elif self.omega_l > 0.01 and self.omega_r < -0.01:
                if self.omega_l < 0.15 and self.omega_r > -0.15:
                    self.omega_l = 0
                    self.omega_r = 0
                else:
                    self.omega_l -= 0.15
                    self.omega_r += 0.15

        # limit self.omega_l to [-self.omega_max, self.omega_max]:
        if self.omega_l > self.omega_max:
            self.omega_l = self.omega_max
        elif self.omega_l < -self.omega_max:
            self.omega_l = -self.omega_max

        # limit self.omega_r to [-self.omega_max, self.omega_max]:
        if self.omega_r > self.omega_max:
            self.omega_r = self.omega_max
        elif self.omega_r < -self.omega_max:
            self.omega_r = -self.omega_max

        ctrl_msg = Float64MultiArray()
        ctrl_msg.data = [self.omega_l, self.omega_r]

        return ctrl_msg

    # define a member function which computes new control signals and sends
    # these to the robot with a frequency of 10 Hz:
    def run(self):
        # specify the desired loop frequency in Hz:
        rate = rospy.Rate(10)

        while not rospy.is_shutdown(): # (while the ROS node is still active:)
            # compute new control signals:
            ctrl_msg = self.get_control_signals()

            # publish the control signals:
            self.pub.publish(ctrl_msg)

            # sleep to get a loop frequency of 10 Hz:
            rate.sleep()

if __name__ == "__main__":
    # create an instance of the Manual_controller class (this will run
    # its __init__ function):
    manual_controller = Manual_controller()

    # run the member fuction run defined above:
    manual_controller.run()
