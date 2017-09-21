#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publisher():
    # create a publisher that publishes messages of type String on the
    # topic /test_topic:
    pub = rospy.Publisher("/test_topic", String, queue_size=10)

    # initialize this code as a ROS node named publisher_node:
    rospy.init_node("publisher_node", anonymous=True)

    # specify the desired loop frequency in Hz:
    rate = rospy.Rate(1)

    while not rospy.is_shutdown(): # (while the ROS node is still active:)
        # specify the string that you want to send/publish:
        message = "Hej!"

        # publish the message (on the specified topic, i.e., /test_topic):
        pub.publish(message)

        # sleep to get a loop frequency of 1 Hz:
        rate.sleep()

if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
