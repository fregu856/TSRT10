#!/usr/bin/env python

import rospy
from scipy.ndimage.measurements import label
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

import numpy as np
import cv2

import time

def frontier_func(slamMap, currentPosition, map_msg):
    rows, cols = slamMap.shape
    frontierMap = np.zeros(shape=(rows,cols))
    #print slamMap

    # save slamMap as an image:
    map_height, map_width = slamMap.shape
    img = np.zeros((map_height, map_width, 3))
    for row in range(map_height):
        for col in range(map_width):
            point = slamMap[row][col]
            if point == -1:
                img[row][col] = [100, 100, 100]
            elif point == 0:
                img[row][col] = [255, 255, 255]
            elif point == 100:
                img[row][col] = [0, 0, 0]
    cv2.imwrite("test_frontier.png", img)

    # set all nodes outside of the 8x8 square to 100 (= obstacle):
    x_min = -4
    x_max = 4
    y_min = -4
    y_max = 4
    #
    temp = pos_2_map_index(map_msg, [x_min, y_min])
    x_min_ind = temp[0]
    y_min_ind = temp[1]
    temp = pos_2_map_index(map_msg, [x_max, y_max])
    x_max_ind = temp[0]
    y_max_ind = temp[1]
    #
    if x_max_ind < cols:
        slamMap[:, x_max_ind:] = 100
    if x_min_ind > 0:
        slamMap[:, 0:x_min_ind] = 100
    if y_max_ind < rows:
        slamMap[y_max_ind:, :] = 100
    if y_min_ind > 0:
        slamMap[0:y_min_ind, :] = 100

    # save slamMap as an image after bordering:
    map_height, map_width = slamMap.shape
    img = np.zeros((map_height, map_width, 3))
    for row in range(map_height):
        for col in range(map_width):
            point = slamMap[row][col]
            if point == -1:
                img[row][col] = [100, 100, 100]
            elif point == 0:
                img[row][col] = [255, 255, 255]
            elif point == 100:
                img[row][col] = [0, 0, 0]
    cv2.imwrite("test_frontier_border.png", img)

    # # filter lone obstacle points:
    # n_threshold = 1
    # slamMap_binary = np.zeros((rows, cols))
    # slamMap_binary[slamMap == 100] = 1
    # labeled_array, num_features = label(slamMap_binary)
    # binc = np.bincount(labeled_array.ravel())
    # noise_idx = np.where(binc <= n_threshold)
    # shp = slamMap.shape
    # mask = np.in1d(labeled_array, noise_idx).reshape(shp)
    # slamMap[mask] = 0
    #
    # # save slamMap as an image after obstacle filtering:
    # map_height, map_width = slamMap.shape
    # img = np.zeros((map_height, map_width, 3))
    # for row in range(map_height):
    #     for col in range(map_width):
    #         point = slamMap[row][col]
    #         if point == -1:
    #             img[row][col] = [100, 100, 100]
    #         elif point == 0:
    #             img[row][col] = [255, 255, 255]
    #         elif point == 100:
    #             img[row][col] = [0, 0, 0]
    # cv2.imwrite("test_frontier_filtered.png", img)

    # expand all obstacles:
    obst_inds = np.nonzero(slamMap == 100)
    obst_inds_row = obst_inds[0].tolist()
    obst_inds_col = obst_inds[1].tolist()
    obst_inds = zip(obst_inds_row, obst_inds_col)
    for obst_ind in obst_inds:
        obst_row = obst_ind[0]
        obst_col = obst_ind[1]
        for row in range(obst_row-6, obst_row+7):
            for col in range(obst_col-6, obst_col+7):
                if row < rows and col < cols:
                    slamMap[row][col] = 100

    # save slamMap as an image after obstacle expansion:
    map_height, map_width = slamMap.shape
    img = np.zeros((map_height, map_width, 3))
    for row in range(map_height):
        for col in range(map_width):
            point = slamMap[row][col]
            if point == -1:
                img[row][col] = [100, 100, 100]
            elif point == 0:
                img[row][col] = [255, 255, 255]
            elif point == 100:
                img[row][col] = [0, 0, 0]
    cv2.imwrite("test_frontier_expanded.png", img)

    slamMapNoObstacles = slamMap
    slamMapNoObstacles[slamMapNoObstacles == -2] = 0    #Covered
    slamMapNoObstacles[slamMapNoObstacles == 100] = 0   #Obstacle]
    #print slamMapNoObstacles

    tempMap1 = np.zeros((rows+1, cols+1))
    tempMap1[1:, 1:] = slamMapNoObstacles
    #print tempMap1

    tempMap2 = np.zeros((rows+1, cols+1))
    tempMap2[0:-1, 0:-1] = slamMapNoObstacles
    #print tempMap2

    frontierNodes = tempMap2 - tempMap1 ## 16x16
    #print frontierNodes

    FirstMap = frontierNodes[0:-1,0:-1]
    #print FirstMap

    SecondMap = frontierNodes[1:,1:]
    #print SecondMap

    frontierMap[FirstMap == -1] = 1
    frontierMap[SecondMap == 1] = 1
    #print frontierMap

    tempMap3 = np.zeros((rows+1, cols+1))
    tempMap3[0:-1, 1:] = slamMapNoObstacles
    #print tempMap3

    tempMap4 = np.zeros((rows+1, cols+1))
    tempMap4[1:, 0:-1] = slamMapNoObstacles
    #print tempMap4

    frontierNodes = tempMap3 - tempMap4
    #print frontierNodes

    FirstMap = frontierNodes[:-1,1:]
    SecondMap = frontierNodes[1:,0:-1]
    frontierMap[FirstMap == -1] = 1
    frontierMap[SecondMap == 1] = 1
    #print frontierMap

    # Remove covered area and obstacles as frontier candidates:
    frontierMap[slamMap == 100] = 0
    frontierMap[slamMap == -2] = 0

    # filter lone frontier nodes:
    n_threshold = 3
    labeled_array, num_features = label(frontierMap)
    binc = np.bincount(labeled_array.ravel())
    noise_idx = np.where(binc <= n_threshold)
    shp = frontierMap.shape
    mask = np.in1d(labeled_array, noise_idx).reshape(shp)
    frontierMap[mask] = 0
    #print frontierMap

    # remove nodes that are too close to the current node:
    for row in range(currentPosition[1]-6, currentPosition[1]+7):
        for col in range(currentPosition[0]-6, currentPosition[0]+7):
            if row < rows and col < cols:
                frontierMap[row][col] = 0
    #print frontierMap

    temp = np.nonzero(frontierMap)
    x = temp[1]
    y = temp[0]
    if len(x) > 0:
        goalNode = np.argmin((x-currentPosition[0])**2 + (y-currentPosition[1])**2)
        return [x[goalNode], y[goalNode]] # ([x (col), y (row)])
    else:
        while True:
            print "frontier is done!"

def map_index_2_pos(map_msg, pos_index):
    map_origin_obj = map_msg.info.origin
    map_origin = [map_origin_obj.position.x, map_origin_obj.position.y]

    map_resolution = map_msg.info.resolution

    map_height = map_msg.info.height

    x_map_ind = pos_index[0]
    y_map_ind = pos_index[1]

    x_map = x_map_ind*map_resolution
    y_map = y_map_ind*map_resolution

    x = x_map + map_origin[0]
    y = y_map + map_origin[1]

    pos = [x, y]

    return pos

def pos_2_map_index(map_msg, pos):
    map_origin_obj = map_msg.info.origin
    map_origin = [map_origin_obj.position.x, map_origin_obj.position.y]

    map_resolution = map_msg.info.resolution

    map_height = map_msg.info.height

    x = pos[0]
    y = pos[1]

    x_map = x - map_origin[0]
    y_map = y - map_origin[1]

    x_map_ind = int(x_map/map_resolution) # (col)
    y_map_ind = int(y_map/map_resolution) # (row)

    pos_index = [x_map_ind, y_map_ind]

    return pos_index

def map_msg_2_matrix(map_msg):
    map_data = map_msg.data
    map_height = map_msg.info.height

    # convert the map data from a list into a numpy array:
    map_matrix = np.array(map_data)
    # split the array into a list of map_height arrays (each list element is a row):
    map_matrix = np.split(map_matrix, map_height)
    # convert the list of arrays into a 2D array:
    map_matrix = np.array(map_matrix)

    return map_matrix

class Frontier:
    def __init__(self):
        # initialize this code as a ROS node named nav_frontier_node:
        rospy.init_node("nav_frontier_node", anonymous=True)

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        rospy.Subscriber("/estimated_pose", Float64MultiArray, self.est_pose_callback)

        # subscribe to the topic onto which the controller publishes data once
        # it has reached a goal position:
        rospy.Subscriber("/controller_status", String, self.controller_callback)

        # create a publisher for publishing goal positions to the controller:
        self.goal_pos_pub = rospy.Publisher("/goal_pos", Float64MultiArray, queue_size=10)

        self.map_msg = None
        self.x = None
        self.y = None
        self.theta = None


        # get things moving (seems like we need to wait a short moment for the
        # message to actually be published):
        msg = Float64MultiArray()
        msg.data = [0, 0.1]
        time.sleep(0.5)
        self.goal_pos_pub.publish(msg)

        print "hej"

        # keep python from exiting until this ROS node is stopped:
        rospy.spin()

    # define the callback function for the /map subscriber:
    def map_callback(self, msg_obj):
        self.map_msg = msg_obj

        print "map_callback"

    # define the callback function for the /estimated_pose subscriber:
    def est_pose_callback(self, msg_obj):
        pose = msg_obj.data

        self.x = pose[0]
        self.y = pose[1]
        self.theta = pose[2]

        print "est_pose_callback"

    # define the callback function for the /controller_status subscriber:
    def controller_callback(self, msg_obj):
        # get the recived string:
        msg_string = msg_obj.data

        if msg_string == "reached target position":
            goal_pos = self.get_goal_pos()
            if goal_pos is not None:
                # publish the goal position:
                msg = Float64MultiArray()
                msg.data = goal_pos
                self.goal_pos_pub.publish(msg)

    def get_goal_pos(self):
        if self.map_msg is not None and self.x is not None and self.y is not None:
            map_msg = self.map_msg
            x = self.x
            y = self.y
            pos = [x, y]

            map_matrix = map_msg_2_matrix(map_msg)
            #print map_matrix
            #print map_matrix.shape

            pos_index = pos_2_map_index(map_msg, pos)
            print "current pos:"
            print pos
            print "current pos index:"
            print pos_index
            #print map_index_2_pos(map_msg, pos_index)

            goal_pos_index = frontier_func(map_matrix, pos_index, map_msg)
            print "goal pos index:"
            print goal_pos_index

            goal_pos = map_index_2_pos(map_msg, goal_pos_index)
            print "goal pos:"
            print goal_pos
            print "##################################################"

            return goal_pos
        else:
            return None

if __name__ == "__main__":
    # create a Frontier object (this will run its __init__ function):
    frontier = Frontier()
