#!/usr/bin/env python

# in this file we read frames from the RPI camera, convert them to
# sensor_msgs/Image and publish them on the topic /balrog_camera/image_raw.

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

if __name__ == '__main__':
    # initialize the ROS node:
    rospy.init_node("stream_to_ROS_node", anonymous=True)

    # create publisher to publish frames from the vido stream:
    image_pub = rospy.Publisher("/balrog_camera/image_raw", Image, queue_size=10)

    # initialize cv_bridge for conversion between openCV and ROS images:
    cv_bridge = CvBridge()

    # connect to the RPI video stream:
    cap = cv2.VideoCapture("tcp://10.0.0.10:8080")

    while not rospy.is_shutdown():
        # read the latest frame from the video stream:
        ret, frame = cap.read()
        img_cv = frame

        # # Display the resulting frame
        # cv2.imshow("test", img_cv)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

        # convert the video frame from openCV format to ROS format:
        img_ROS_msg = cv_bridge.cv2_to_imgmsg(img_cv, "bgr8")

        # publish the image in ROS format:
        image_pub.publish(img_ROS_msg)
