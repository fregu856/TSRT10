#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Manual_controller:
    def __init__(self):
        rospy.init_node("manual_controller_node", anonymous=True)

        self.pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)

        rospy.Subscriber("/steering_cmds", String, self.cmd_callback)

        self.throttle_direction = "No_throttle"
        self.steering_direction = "No_steering"

        self.v = 0
        self.omega = 0

        self.v_max = 0.65
        self.omega_max = 1.5

    def cmd_callback(self, msg_obj):
        cmd = msg_obj.data

        if cmd in ["Forward", "Backward", "No_throttle"]:
            self.throttle_direction = cmd

        elif cmd in ["Right", "Left", "No_steering"]:
            self.steering_direction = cmd

    def get_control_signals(self):
        if self.steering_direction == "Right":
            if self.omega < - 0.001:
                self.omega = 0
            else:
                self.omega = self.omega + 0.3
        elif self.steering_direction == "Left":
            if self.omega > 0.001:
                self.omega = 0
            else:
                self.omega = self.omega - 0.3
        elif self.steering_direction == "No_steering":
            self.omega = 0

        if self.omega > self.omega_max:
            self.omega = self.omega_max
        elif self.omega < -self.omega_max:
            self.omega = -self.omega_max


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

        # # limit the throttle to [throttle_min, throttle_max]:
        if self.v > self.v_max:
            self.v = self.v_max
        elif self.v < -self.v_max:
            self.v = -self.v_max


        control_signals = Twist()
        control_signals.linear.x = self.v
        control_signals.angular.z = -self.omega

        return control_signals

    def run(self):
        rate = rospy.Rate(10) # (10 Hz)
        while not rospy.is_shutdown():
            #print self.throttle_direction
            #print self.steering_direction
            #print self.v
            #print -self.omega

            control_signals = self.get_control_signals()
            self.pub.publish(control_signals)
            rate.sleep() # (to get loop freq. of 10 Hz)

if __name__ == "__main__":
    manual_controller = Manual_controller()
    manual_controller.run()
