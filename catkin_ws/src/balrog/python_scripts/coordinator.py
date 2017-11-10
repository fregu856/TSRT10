#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

import numpy as np

class Coordinator:
    def __init__(self):
        # initialize this code as a ROS node named coordinator_node:
        rospy.init_node("coordinator_node", anonymous=True)

        # subscribe to the topic containing the path outputted by A*:
        rospy.Subscriber("/path", Float64MultiArray, self.path_callback)

        # subscribe to the topic onto which the controller publishes data once
        # it has reached a goal position:
        rospy.Subscriber("/controller_status", String, self.controller_callback)

        # create a publisher for publishing goal positions to the controller:
        self.goal_pos_pub = rospy.Publisher("/goal_pos", Float64MultiArray, queue_size=10)

        self.status_pub = rospy.Publisher("/coordinator_status", String, queue_size=10)

        #self.path = [[0,0], [0.5,0], [0.5,0.5], [0,0.5], [0,0]]
        self.path = [[0,0]]
        self.next_index = 0 # (list index of the next goal position to be published)
        self.final_index = len(self.path) - 1 # (index of the last path list element)

        # keep python from exiting until this ROS node is stopped:
        rospy.spin()

    # define the callback function for the /path subscriber:
    def path_callback(self, msg_obj):
        print "received new path!"

        # get the published path:
        path_list = msg_obj.data

        path = []
        for i in range(len(path_list)):
            if i % 2 == 0:
                path.append([path_list[i], path_list[i+1]])

        self.path = path
        print self.path
        self.next_index = 0
        self.final_index = len(self.path) - 1

        # get the first goal position on the path:
        goal_pos = self.path[0]
        # publish the goal position:
        msg = Float64MultiArray()
        msg.data = goal_pos
        self.goal_pos_pub.publish(msg)

        self.next_index += 1

        print "path_callback published new goal position, x: %f, y: %f" % (goal_pos[0], goal_pos[1])

    # define the callback function for the /controller_status subscriber:
    def controller_callback(self, msg_obj):
        # get the recived string:
        msg_string = msg_obj.data

        if msg_string == "reached target position":
            if self.next_index <= self.final_index:
                # get the next goal position:
                goal_pos = self.path[self.next_index]
                # publish the goal position:
                msg = Float64MultiArray()
                msg.data = goal_pos
                self.goal_pos_pub.publish(msg)

                self.next_index += 1

                print "published new goal position, x: %f, y: %f" % (goal_pos[0], goal_pos[1])
            else:
                if self.next_index == self.final_index + 1:
                    print "reached end of path"
                    msg = "reached end of path"
                    self.status_pub.publish(msg)
                else:
                    print "waiting for new path"

                self.next_index += 1

if __name__ == "__main__":
    # create a Coordinator object (this will run its __init__ function):
    ctrl = Coordinator()
