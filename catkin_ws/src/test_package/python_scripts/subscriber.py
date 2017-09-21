#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def subscriber():
    # initialize this code as a ROS node named subscriber_node:
    rospy.init_node("subscriber_node", anonymous=True)

    # subscribe to the topic /test_topic. That is, read every message (of type
    # String) that is published on /test_topic. We will do this by calling the
    # function callback_function everytime a new message is published:
    rospy.Subscriber("/test_topic", String, callback_function)

    # keep python from exiting until this ROS node is stopped:
    rospy.spin()

# define the callback function for the /test_topic subscriber:
def callback_function(message_object):
    # get the actual message String that was published:
    received_message = message_object.data

    # print the received String:
    print received_message

if __name__ == "__main__":
    subscriber()
