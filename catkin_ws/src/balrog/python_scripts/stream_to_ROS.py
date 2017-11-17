#!/usr/bin/env python

# in this file we read frames from the RPI camera, convert them to
# sensor_msgs/Image and publish them on the topic /balrog_camera/image_raw.

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from threading import Thread

class Streamer:
    def __init__(self):
        # initialize the ROS node:
        rospy.init_node("stream_to_ROS_node", anonymous=True)

        # create publisher to publish frames from the vido stream:
        self.image_pub = rospy.Publisher("/balrog_camera/image_raw", Image, queue_size = 1)

        # initialize cv_bridge for conversion between openCV and ROS images:
        self.cv_bridge = CvBridge()

        self.latest_frame =  None

        # start a thread constantly reading frames from the RPI video stream:
        self.thread_video = Thread(target = self.video_thread)
        self.thread_video.start()

    def video_thread(self):
        print "connecting:"
        # connect to the RPI video stream:
        cap = cv2.VideoCapture("tcp://10.0.0.10:8080")
        print "connected!"

        while True:
            # capture frame-by-frame:
            ret, frame = cap.read()
            self.latest_frame = frame

    def run(self):
        rate = rospy.Rate(10) # (10 Hz)
        counter = 0
        while not rospy.is_shutdown():
            if self.latest_frame is not None:
                print counter
                counter += 1

                # convert the video frame from openCV format to ROS format:
                img_ROS_msg = self.cv_bridge.cv2_to_imgmsg(self.latest_frame, "bgr8")

                # publish the image in ROS format:
                self.image_pub.publish(img_ROS_msg)

                rate.sleep() # (to get it to loop with 10 Hz)

if __name__ == '__main__':
    streamer = Streamer()

    streamer.run()
