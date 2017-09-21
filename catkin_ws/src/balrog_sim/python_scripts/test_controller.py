#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import tf
import numpy as np

class Controller:

    def __init__(self):
        rospy.init_node("test_controller", anonymous=True)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        self.pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def callback(self, data):
        pose = data.pose[data.name.index("mobile_base")]
        twist = data.twist[data.name.index("mobile_base")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def wrapToPi(self, a):
        if isinstance(a, list):    # backwards compatibility for lists (distinct from np.array)
            return [(x + np.pi) % (2*np.pi) - np.pi for x in a]
        return (a + np.pi) % (2*np.pi) - np.pi

    def get_ctrl_output(self):
        cmd_x_dot = 0.0 # (forward velocity = v)
        cmd_theta_dot = 0.0 # (omega)

        # set a goal x, y, theta:
        x_g = 1
        y_g = 1
        th_g = 0

        # controller params:
        k1 = 0.5
        k2 = 0.5
        k3 = 0.5

        rho = np.sqrt((self.x - x_g)**2 + (self.y - y_g)**2)
        alpha = np.arctan2(self.y - y_g, self.x - x_g) - self.theta + np.pi
        alpha = self.wrapToPi(alpha)
        delta = np.arctan2(self.y - y_g, self.x - x_g) + np.pi - th_g
        delta = self.wrapToPi(delta)

        # compute control outputs v and omega:
        v = k1*rho*np.cos(alpha)
        omega = k2*alpha+k1*np.sinc(alpha/np.pi)*np.cos(alpha)*(alpha+k3*delta)

        # apply saturation limits:
        v = np.sign(v)*min(0.5, np.abs(v))
        omega = np.sign(omega)*min(1, np.abs(omega))

        cmd_x_dot = v
        cmd_theta_dot = omega

        cmd = Twist()
        cmd.linear.x = cmd_x_dot
        cmd.angular.z = cmd_theta_dot

        return cmd

    def run(self):
        rate = rospy.Rate(10) # (10 Hz)
        while not rospy.is_shutdown():
            ctrl_output = self.get_ctrl_output()
            self.pub.publish(ctrl_output)
            rate.sleep() # (to get it to loop with 10 Hz)

if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()
