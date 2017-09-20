#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publisher():
    pub = rospy.Publisher("/test_topic", String, queue_size=10)
    rospy.init_node("publisher", anonymous=True)
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        msg = "Hej Hampus!"
        pub.publish(msg)
        rate.sleep() # (to get loop freq. of 1 Hz)

if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
