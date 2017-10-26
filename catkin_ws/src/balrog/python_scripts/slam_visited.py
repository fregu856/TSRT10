#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

from nav_msgs.msg import OccupancyGrid

from visualization_msgs.msg import Marker

import tf
import numpy as np

import cv2

most_recent_map = np.zeros(())
most_recent_map_origin = []
map_resolution = 0.05
most_recent_map_width = 0
most_recent_map_height = 0

pub = rospy.Publisher("/map_visited", OccupancyGrid, queue_size=10)

def callback_func(msg_obj):
    points = msg_obj.points

    map_local = most_recent_map
    map_origin_local = most_recent_map_origin
    map_width_local = most_recent_map_width
    map_height_local = most_recent_map_height

    print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
    print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
    print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
    print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
    print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
    print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
    print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
    print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
    print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
    print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
    print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
    print "new marker array"

    if most_recent_map_origin != []:
        # convert the map to an image and save to disk (for visualization):
        img = np.zeros((map_height_local, map_width_local, 3))
        for row in range(map_height_local):
            for col in range(map_width_local):
                point = map_local[row][col]
                if point == -1:
                    img[row][col] = [100, 100, 100]
                elif point == 0:
                    img[row][col] = [255, 255, 255]
                elif point == 100:
                    img[row][col] = [0, 0, 0]

        for point in points:
            x = point.x
            y = point.y

            x_map = x - map_origin_local[0]
            y_map = y - map_origin_local[1]

            x_map_ind = int(x_map // 0.05) # (col)
            y_map_ind = int(y_map // 0.05) # (row)

            print map_local[y_map_ind][x_map_ind]

            #img[y_map_ind][x_map_ind] = [0, 255, 0]

            for row in range(y_map_ind-10, y_map_ind+11):
                for col in range(x_map_ind-10, x_map_ind+11):
                    if row < map_height_local and col < map_width_local:
                        map_point = map_local[row][col]
                        if map_point == 0:
                            img[row][col] = [0, 255, 0]


            print "x: %f, y: %f" % (x, y)
            print "origin: (%f, %f)" % (map_origin_local[0], map_origin_local[1])
            print "x_map: %f, y_map: %f" % (x_map, y_map)
            print "x_map_ind: %d, y_map_ind: %d" % (x_map_ind, y_map_ind)
            print "##############################################"

        cv2.imwrite("test.png", img)

def callback_func_map(msg_obj):
    global most_recent_map, most_recent_map_origin, most_recent_map_width
    global most_recent_map_height

    #print msg_obj

    map_data = msg_obj.data
    map_width = msg_obj.info.width
    map_height = msg_obj.info.height
    map_origin = msg_obj.info.origin.position
    map_origin = [map_origin.x, map_origin.y]

    # convert the map data into a numpy array:
    map_matrix = np.array(map_data)
    # split the array into a list of map_height arrays (a list containing each row):
    map_matrix = np.split(map_matrix, map_height)
    # convert the list of array into a 2D array:
    map_matrix = np.array(map_matrix)

    most_recent_map = map_matrix
    most_recent_map_origin = map_origin
    most_recent_map_height = map_height
    most_recent_map_width = map_width

    print "width: %d, height: %d" % (map_width, map_height)
    print map_matrix.shape

    # # convert the map to an image and save to disk (for visualization):
    # img = np.zeros((map_height, map_width, 3))
    # for row in range(map_height):
    #     for col in range(map_width):
    #         point = map_matrix[row][col]
    #         if point == -1:
    #             img[row][col] = [100, 100, 100]
    #         elif point == 0:
    #             img[row][col] = [255, 255, 255]
    #         elif point == 100:
    #             img[row][col] = [0, 0, 0]
    # cv2.imwrite("test.png", img)




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
