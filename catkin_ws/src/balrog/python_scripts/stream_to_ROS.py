#!/usr/bin/env python

# in this file we read frames from the RPI camera, convert them to
# sensor_msgs/Image and publish them on the topic /balrog_camera/image_raw.

import rospy
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge

from threading import Thread

class Streamer:
    def __init__(self):
        # initialize the ROS node:
        rospy.init_node("stream_to_ROS_node", anonymous=True)

        # create publisher to publish frames from the video stream:
        self.image_pub = rospy.Publisher("/balrog_camera/image_raw", Image, queue_size = 1)

        # create publisher to publish camera info (calibration params):
        self.camera_info_pub = rospy.Publisher("/balrog_camera/camera_info", CameraInfo, queue_size = 1)

        # initialize cv_bridge for conversion between openCV and ROS images:
        self.cv_bridge = CvBridge()

        self.latest_frame =  None

        # start a thread constantly reading frames from the RPI video stream:
        self.thread_video = Thread(target = self.video_thread)
        self.thread_video.start()

        camera_info_msg = CameraInfo()
        camera_info_msg.height = 360
        camera_info_msg.width = 640
        camera_info_msg.distortion_model = "plumb_bob"
        camera_info_msg.D = [1.6541397736403166e-01, -3.1762679367786140e-01, 0.0, 0.0, -1.4358526468495059e-01] # ([k1, k2, t1, t2, k3])
        camera_info_msg.K = [4.8488441935486514e+02, 0, 320, 0, 4.8488441935486514e+02, 180, 0, 0, 1]  # ([fx, 0, cx, 0, fy, cy, 0, 0, 1])
        camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info_msg = camera_info_msg

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
        while not rospy.is_shutdown():
            if self.latest_frame is not None:
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = "balrog_camera"

                # convert the frame from openCV format to ROS format:
                img_ROS_msg = self.cv_bridge.cv2_to_imgmsg(self.latest_frame, "bgr8")

                img_ROS_msg.header = header
                self.camera_info_msg.header = header

                # publish the frame in ROS format:
                self.image_pub.publish(img_ROS_msg)

                # publish the camera info:
                self.camera_info_pub.publish(self.camera_info_msg)

                rate.sleep() # (to get it to loop with 10 Hz)

if __name__ == '__main__':
    streamer = Streamer()

    streamer.run()

    streamer.thread_video.join()
