#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

from nav_msgs.msg import OccupancyGrid

from visualization_msgs.msg import Marker

import tf
import numpy as np

import cv2

pub = rospy.Publisher("/map_visited", OccupancyGrid, queue_size=10)

def callback_func(msg_obj):
    points = msg_obj.points

    #print "new marker array"

    for point in points:
        x = point.x
        y = point.y

        #print x
        #print y

def callback_func_map(msg_obj):
    map_data = msg_obj.data
    map_width = msg_obj.info.width
    map_height = msg_obj.info.height

    # convert the map data into a numpy array:
    test = np.array(map_data)
    # split the array into a list of map_height arrays (a list containign each row):
    test = np.split(test, map_height)
    # convert the list of array into a 2D array:
    test = np.array(test)

    print "width: %d, height: %d" % (map_width, map_height)
    print test.shape

    # convert the map to an image and save to disk (for visualization):
    img = np.zeros((map_height, map_width, 3))
    for row in range(map_height):
        for col in range(map_width):
            point = test[row][col]
            if point == -1:
                img[row][col] = [100, 100, 100]
            elif point == 0:
                img[row][col] = [255, 255, 255]
            elif point == 100:
                img[row][col] = [0, 0, 0
    cv2.imwrite("test.png", img)


    #print map_data
    #print map_width
    #print map_height




if __name__ == "__main__":
    # initialize this code as a ROS node named slam_visited:
    rospy.init_node("slam_visited", anonymous=True)

    # subscribe to the topic /encoder_data. That is, read every message (of type
    # Float64MultiArray) that is published on /encoder. We will do this by
    # calling the function callback_func every time a new message is published:
    rospy.Subscriber("/Mapper/vertices", Marker, callback_func)

    rospy.Subscriber("/map", OccupancyGrid, callback_func_map)

    # keep python from exiting until this ROS node is stopped:
    rospy.spin()
