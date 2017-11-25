#!/usr/bin/env python

# NOTE! This is just a copy of balrog/python_scripts/slam_visited.py (the python
# version is used in the simulator only because it's slightly more convenient).

# NOTE! This code is a python version of balrog/src/slam_visited.cpp and is not
# used. It is however saved for future reference.

import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

import numpy as np
import cv2

import time

# -1: unknown
# 0: free
# 100: obstacle
# -2: visited

most_recent_map = np.zeros(())
most_recent_map_origin = []
map_resolution = 0.05

pub = rospy.Publisher("/map_visited", OccupancyGrid, queue_size=10)

def callback_func(msg_obj):
    #start = time.time()
    if most_recent_map_origin != []:
        points = msg_obj.points

        map_local = np.copy(most_recent_map)
        map_origin_local_obj = most_recent_map_origin
        map_origin_local = [map_origin_local_obj.position.x, map_origin_local_obj.position.y]

        map_height, map_width = map_local.shape

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

        map_visited = np.copy(map_local)
        for point in points:
            x = point.x
            y = point.y

            x_map = x - map_origin_local[0]
            y_map = y - map_origin_local[1]

            x_map_ind = int(x_map // 0.05) # (col)
            y_map_ind = int(y_map // 0.05) # (row)

            col_ind_max = min(x_map_ind+11, map_width)
            col_ind_min = max(x_map_ind-10, 0)
            row_ind_max = min(y_map_ind+11, map_height)
            row_ind_min = max(y_map_ind-10, 0)
            map_visited[row_ind_min:row_ind_max, col_ind_min:col_ind_max] = -2

            # for row in range(y_map_ind-10, y_map_ind+11):
            #     for col in range(x_map_ind-10, x_map_ind+11):
            #         if row < map_height and row >= 0 and col < map_width and col >= 0:
            #             map_visited[row][col] = -2

            #print "x: %f, y: %f" % (x, y)
            #print "origin: (%f, %f)" % (map_origin_local[0], map_origin_local[1])
            #print "x_map: %f, y_map: %f" % (x_map, y_map)
            #print "x_map_ind: %d, y_map_ind: %d" % (x_map_ind, y_map_ind)
            #print "##############################################"

        msg = OccupancyGrid()
        msg.data = map_visited.flatten().tolist()
        msg.info.resolution = map_resolution
        msg.info.width = map_width
        msg.info.height = map_height
        msg.info.origin = map_origin_local_obj
        pub.publish(msg)

        # # convert map_visited to an image and save to disk (for visualization):
        # img = np.zeros((map_height, map_width, 3))
        # for row in range(map_height):
        #     for col in range(map_width):
        #         point = map_visited[row][col]
        #         if point == -1:
        #             img[row][col] = [100, 100, 100]
        #         elif point == 0:
        #             img[row][col] = [255, 255, 255]
        #         elif point == 100:
        #             img[row][col] = [0, 0, 0]
        #         elif point == -2:
        #             img[row][col] = [0, 255, 0]
        # cv2.imwrite("/home/fregu856/test.png", img)
    #end = time.time()
    #print "marker_callback:"
    #print end - start

def callback_func_map(msg_obj):
    global most_recent_map, most_recent_map_origin

    map_data = msg_obj.data
    map_width = msg_obj.info.width
    map_height = msg_obj.info.height
    map_origin = msg_obj.info.origin

    # convert the map data into a numpy array:
    map_matrix = np.array(map_data)
    # split the array into a list of map_height arrays (a list containing each row):
    map_matrix = np.split(map_matrix, map_height)
    # convert the list of arrays into a 2D array:
    map_matrix = np.array(map_matrix)

    most_recent_map = map_matrix
    most_recent_map_origin = map_origin

    print "width: %d, height: %d" % (map_width, map_height)
    print map_matrix.shape

if __name__ == "__main__":
    # initialize this code as a ROS node named slam_visited:
    rospy.init_node("slam_visited", anonymous=True)

    rospy.Subscriber("/Mapper/vertices", Marker, callback_func)

    rospy.Subscriber("/map", OccupancyGrid, callback_func_map)

    # keep python from exiting until this ROS node is stopped:
    rospy.spin()
