#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

import numpy as np

class Coordinator:
    def __init__(self):
        rospy.init_node("coordinator_node", anonymous=True)

        rospy.Subscriber("/path", Float64MultiArray, self.path_callback)

        rospy.Subscriber("/controller_status", String, self.controller_callback)

        self.goal_pos_pub = rospy.Publisher("/goal_pos", Float64MultiArray, queue_size=10)

        self.path = [[0,0], [0,1], [-1,1], [-2,2], [-2,3], [-1.5, 3.5], [1, 3.5], [1, 2], [0,1], [0,0]]
        self.next_index = 0
        self.final_index = len(self.path) - 1

        rospy.spin()

    def path_callback(self, msg_obj):
        print "received new path!"

        path = msg_obj.data

        self.path = path
        self.next_index = 0
        self.final_index = len(self.path) - 1

        goal_pos = self.path[0]
        msg = Float64MultiArray()
        msg.data = goal_pos
        self.goal_pos_pub.publish(msg)

        self.next_index += 1

        print "published new goal position, x: %f, y: %f" % (goal_pos[0], goal_pos[1])

    def controller_callback(self, msg_obj):
        msg_string = msg_obj.data
        if msg_string == "reached target position":
            if self.next_index <= self.final_index:
                goal_pos = self.path[self.next_index]
                msg = Float64MultiArray()
                msg.data = goal_pos
                self.goal_pos_pub.publish(msg)

                self.next_index += 1

                print "published new goal position, x: %f, y: %f" % (goal_pos[0], goal_pos[1])

if __name__ == "__main__":
    ctrl = Coordinator()
