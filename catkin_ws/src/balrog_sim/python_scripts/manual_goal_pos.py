#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

if __name__ == "__main__":
    # initialize this code as a ROS node named manual_goal_pos_node:
    rospy.init_node("manual_goal_pos_node", anonymous=True)

    pub = rospy.Publisher("/goal_pos", Float64MultiArray, queue_size=10)

    # specify the desired loop frequency in Hz:
    rate = rospy.Rate(10)

    while not rospy.is_shutdown(): # (while the ROS node is still active:)
        user_input = raw_input("Enter x-coordinate: ")
        try:
            x_coord = float(user_input)
        except:
            print("Not a valid x-coordinate entered\n")
            # skip to the next iteration of the while loop:
            continue

        user_input = raw_input("Enter y-coordinate: ")
        try:
            y_coord = float(user_input)
        except:
            print("Not a valid y-coordinate entered\n")
            # skip to the next iteration of the while loop:
            continue

        print("")

        msg = Float64MultiArray()
        goal_pos = [x_coord, y_coord]
        msg.data = goal_pos
        pub.publish(msg)

        # sleep to get the desired loop frequency:
        rate.sleep()
