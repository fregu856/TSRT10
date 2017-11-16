#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

import numpy as np
import cv2
from scipy.ndimage.measurements import label
import time

from utilities import map_index_2_pos, pos_2_map_index, raw_path_2_path, map_msg_2_matrix
from nav_frontier import frontier_func
from nav_astar import astar_func
from nav_covering import coverageMap, find_goal

X_MAX = 4 # (NOTE! if this value is modified one also needs to update it in covering.py)
X_MIN = -4 # (NOTE! if this value is modified one also needs to update it in covering.py)
Y_MAX = 4 # (NOTE! if this value is modified one also needs to update it in covering.py)
Y_MIN = -4 # (NOTE! if this value is modified one also needs to update it in covering.py)

MAP_RES_SLAM = 0.05 # (map resolution)
MAP_RES_ASTAR = 0.25 # (NOTE! if this value is modified one also needs to update it in covering.py)
MAP_RES_COVERING = 0.30 # (NOTE! if this value is modified one also needs to update it in covering.py)

class Mapping:
    def __init__(self):
        # initialize this code as a ROS node named nav_mapping_node:
        rospy.init_node("nav_mapping_node", anonymous=True)

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        rospy.Subscriber("/estimated_pose", Float64MultiArray, self.est_pose_callback)

        # subscribe to the topic onto which the coordinator publishes data once
        # it has reached the end of a path:
        rospy.Subscriber("/coordinator_status", String, self.coordinator_callback)

        rospy.Subscriber("/map_covering", OccupancyGrid, self.map_covering_callback)

        # create a publisher for publishing paths to the coordinator:
        self.path_pub = rospy.Publisher("/path", Float64MultiArray, queue_size=10)
        self.warning_pub = rospy.Publisher("/node_status", String, queue_size=1)
        self.map_msg = None
        self.x = None
        self.y = None
        self.theta = None
        self.goal_pos_index = None
        self.path = None
        self.map_matrix = None
        self.map_matrix_expand = None
        self.map_small = None
        self.mode = 1
        self.map_covering = None

        # get things moving (seems like we need to wait a short moment for the
        # message to actually be published):a
        msg = Float64MultiArray()
        msg.data = [-0.1, 0, 0.1, 0]
        time.sleep(0.5)
        self.path_pub.publish(msg)

        # keep python from exiting until this ROS node is stopped:
        #rospy.spin()
    # define the callback function for the /map subscriber:

    def map_callback(self, msg_obj):
        self.map_msg = msg_obj
        self.map_matrix = map_msg_2_matrix(self.map_msg)
        map_matrix = self.map_matrix
        map_matrix_rows, map_matrix_cols = map_matrix.shape
        map_matrix_expand = np.copy(map_matrix)

        # save map_matrix_expand as an image:
        map_height, map_width = map_matrix_expand.shape
        img = np.zeros((map_height, map_width, 3))
        for row in range(map_height):
            for col in range(map_width):
                point = map_matrix_expand[row][col]
                if point == -1:
                    img[row][col] = [100, 100, 100]
                elif point == 0:
                    img[row][col] = [255, 255, 255]
                elif point == 100:
                    img[row][col] = [0, 0, 0]
        cv2.imwrite("map_matrix_expand1.png", img)

        # # filter lone obstacle points:
        n_threshold = 2
        slamMap_binary = np.zeros((map_matrix_rows, map_matrix_cols))
        slamMap_binary[map_matrix == 100] = 1
        labeled_array, num_features = label(slamMap_binary)
        binc = np.bincount(labeled_array.ravel())
        noise_idx = np.where(binc <= n_threshold)
        shp = map_matrix.shape
        mask = np.in1d(labeled_array, noise_idx).reshape(shp)
        map_matrix_expand[mask] = 0

        # save map_matrix_expand as an image:
        map_height, map_width = map_matrix_expand.shape
        img = np.zeros((map_height, map_width, 3))
        for row in range(map_height):
            for col in range(map_width):
                point = map_matrix_expand[row][col]
                if point == -1:
                    img[row][col] = [100, 100, 100]
                elif point == 0:
                    img[row][col] = [255, 255, 255]
                elif point == 100:
                    img[row][col] = [0, 0, 0]
        cv2.imwrite("map_matrix_expand2.png", img)

        # Obstacle EXPAND:
        temp = np.copy(map_matrix_expand)
        OBSTACLE_EXPAND_SIZE = 11
        obst_inds = np.nonzero(map_matrix_expand == 100)
        obst_inds_row = obst_inds[0].tolist()
        obst_inds_col = obst_inds[1].tolist()
        obst_inds = zip(obst_inds_row, obst_inds_col)
        for obst_ind in obst_inds:
            obst_row = obst_ind[0]
            obst_col = obst_ind[1]
            for row in range(obst_row-OBSTACLE_EXPAND_SIZE, obst_row+OBSTACLE_EXPAND_SIZE+1):
                for col in range(obst_col-OBSTACLE_EXPAND_SIZE, obst_col+OBSTACLE_EXPAND_SIZE+1):
                    if row >= 0 and row < map_matrix_rows and col >= 0 and col < map_matrix_cols:
                        if temp[row][col] != 100:
                            map_matrix_expand[row][col] = 100

        # save map_matrix_expand as an image:
        map_height, map_width = map_matrix_expand.shape
        img = np.zeros((map_height, map_width, 3))
        for row in range(map_height):
            for col in range(map_width):
                point = map_matrix_expand[row][col]
                if point == -1:
                    img[row][col] = [100, 100, 100]
                elif point == 0:
                    img[row][col] = [255, 255, 255]
                elif point == 100:
                    img[row][col] = [0, 0, 0]
        cv2.imwrite("map_matrix_expand3.png", img)

        # set all nodes outside of the 8x8 square to 100 (= obstacle):
        x_min = X_MIN
        x_max = X_MAX
        y_min = Y_MIN
        y_max = Y_MAX
        #
        map_msg = msg_obj
        temp = pos_2_map_index(map_msg, MAP_RES_SLAM, [x_min, y_min])
        x_min_ind = temp[0]
        y_min_ind = temp[1]
        temp = pos_2_map_index(map_msg, MAP_RES_SLAM, [x_max, y_max])
        x_max_ind = temp[0]
        y_max_ind = temp[1]
        #
        if x_max_ind < map_matrix_cols:
            map_matrix_expand[:, x_max_ind:] = 100
        if x_min_ind > 0:
            map_matrix_expand[:, 0:x_min_ind] = 100
        if y_max_ind < map_matrix_rows:
            map_matrix_expand[y_max_ind:, :] = 100
        if y_min_ind > 0:
            map_matrix_expand[0:y_min_ind, :] = 100

        self.map_matrix_expand = map_matrix_expand

        # save map_matrix_expand as an image:
        map_height, map_width = map_matrix_expand.shape
        img = np.zeros((map_height, map_width, 3))
        for row in range(map_height):
            for col in range(map_width):
                point = map_matrix_expand[row][col]
                if point == -1:
                    img[row][col] = [100, 100, 100]
                elif point == 0:
                    img[row][col] = [255, 255, 255]
                elif point == 100:
                    img[row][col] = [0, 0, 0]
        cv2.imwrite("map_matrix_expand4.png", img)

        map_height = map_matrix_rows
        map_width = map_matrix_cols
        # Number of cells to make box of. Size of box = NR_OF_CELLS^2
        NR_OF_CELLS = 5
        MIN_NR_OF_OBSTACLES = 11
        map_small_height = int(map_height / NR_OF_CELLS)
        map_small_width = int(map_width / NR_OF_CELLS)
        #print "Visited map height: %f width: %f" % (map_height, map_width)
        #print "Covering map height: %f width: %f" % (map_small_height, map_small_width)
        map_small = np.zeros((map_small_height, map_small_width))
        #loop in every box
        for box_row in range(0, map_small_height):
            for box_col in range(0 , map_small_width):
                #loop for every cell to be part of box
                firstrow = box_row * NR_OF_CELLS
                firstcol = box_col * NR_OF_CELLS
                current_cell = 0
                nr_of_obstacles = 0
                for row in range(firstrow, firstrow + NR_OF_CELLS):
                    for col in range(firstcol , firstcol + NR_OF_CELLS):
                        point = map_matrix_expand[row][col]
                        # Obstacle in current_cell, mark obstacle always
                        if point == 100:
                            # Filter to check for more than MIN_NR_OF_OBSTACLES nodes with obstacle
                            if nr_of_obstacles <= MIN_NR_OF_OBSTACLES:
                                nr_of_obstacles += 1
                            else:
                                current_cell = 100
                # Write to covered_tiles_map
                map_small[box_row][box_col] = current_cell
        self.map_small = map_small
        map_height, map_width = map_small.shape
        img = np.zeros((map_height, map_width, 3))
        for row in range(map_height):
            for col in range(map_width):
                point = map_small[row][col]
                if point == -1:
                    img[row][col] = [100, 100, 100]
                elif point == 0:
                    img[row][col] = [255, 255, 255]
                elif point == 100:
                    img[row][col] = [0, 0, 0]
        cv2.imwrite("map_small.png", img)






        # whites = np.count_nonzero(np.nonzero(map_matrix_expand == 0))
        # grays = np.count_nonzero(np.nonzero(map_matrix_expand == -1))
        # percentage = float(whites)/float(whites + grays)
        # print percentage
        # if percentage > 0.98:
        #     print "HALLA ELLLER!"






        print "map_callback"

    def map_covering_callback(self, msg_obj):
        map_data = msg_obj.data
        map_width = msg_obj.info.width
        map_height = msg_obj.info.height

        # convert the map data into a numpy array:
        map_matrix = np.array(map_data)
        # split the array into a list of map_height arrays (a list containing each row):
        map_matrix = np.split(map_matrix, map_height)
        # convert the list of array into a 2D array:
        map_matrix = np.array(map_matrix)

        self.map_covering = map_matrix

    # define the callback function for the /estimated_pose subscriber:
    def est_pose_callback(self, msg_obj):
        pose = msg_obj.data
        self.x = pose[0]
        self.y = pose[1]
        self.theta = pose[2]
        #print "est_pose_callback"

    # define the callback function for the /coordinator_status subscriber:
    def coordinator_callback(self, msg_obj):
        print "start of coordinator_callback"

        # get the recived string:
        msg_string = msg_obj.data

        if msg_string == "reached end of path":
            path = self.get_path()
            if path is not None:
                # publish the path:
                msg = Float64MultiArray()
                msg.data = path
                self.path_pub.publish(msg)
            else:
                print "path is None!"

        print "end of coordinator_callback"

    def get_path(self):
        if self.map_msg is not None and self.x is not None and self.y is not None:
            print "****************************************************"
            print "start of get_path()"
            map_msg = self.map_msg
            x = self.x
            y = self.y
            pos = [x, y]
            map_matrix = self.map_matrix_expand
            map_matrix_small = self.map_small
            pos_index = pos_2_map_index(map_msg, MAP_RES_SLAM, pos)
            pos_index_small = pos_2_map_index(map_msg, MAP_RES_ASTAR, pos)

            print "pos:", pos
            print "pos index:", pos_index


            if self.mode==1:
                goal_pos_index = frontier_func(np.copy(map_matrix), pos_index, map_msg)
                if goal_pos_index==[-1000, -1000]:
                    self.mode=2
                    path = self.get_path()
                    return path
                else:
                    self.goal_pos_index = goal_pos_index
                    goal_pos = map_index_2_pos(map_msg, MAP_RES_SLAM, goal_pos_index)
                    print "goal index in FRONTIER map:", goal_pos_index
                    print "goal pos:", goal_pos
                    print "value of goal node in FRONTIER map:", map_matrix[goal_pos_index[1], goal_pos_index[0]]

                    goal_pos_index_small = pos_2_map_index(map_msg, MAP_RES_ASTAR, goal_pos)
                    print "goal index in ASTAR map:", goal_pos_index_small
                    print "goal pos corresponding to goal index in ASTAR map:", map_index_2_pos(map_msg, MAP_RES_ASTAR, goal_pos_index_small)
                    print "value of goal node in ASTAR map:", map_matrix_small[goal_pos_index_small[1], goal_pos_index_small[0]]

                    if map_matrix_small[goal_pos_index_small[1], goal_pos_index_small[0]] == 100:
                        print "########################################################"
                        print "get_path(): GOAL node is not free, get the closest free node!"
                        print "########################################################"
                        temp = np.nonzero(map_matrix_small == 0)
                        x = temp[1]
                        y = temp[0]
                        distances = (x-goal_pos_index_small[0])**2 + (y-goal_pos_index_small[1])**2
                        goalNode = np.argmin(distances)
                        goal_pos_index_small = [x[goalNode], y[goalNode]]

                        print "goal index in ASTAR map:", goal_pos_index_small
                        print "goal pos corresponding to goal index in ASTAR map:", map_index_2_pos(map_msg, MAP_RES_ASTAR, goal_pos_index_small)
                        print "value of goal node in ASTAR map:", map_matrix_small[goal_pos_index_small[1], goal_pos_index_small[0]]

                    if map_matrix_small[pos_index_small[1], pos_index_small[0]] == 100:
                        print "########################################################"
                        print "get_path(): START node is not free, get the closest free node!"
                        print "########################################################"
                        temp = np.nonzero(map_matrix_small == 0)
                        x = temp[1]
                        y = temp[0]
                        distances = (x-pos_index_small[0])**2 + (y-pos_index_small[1])**2
                        startNode = np.argmin(distances)
                        pos_index_small = [x[startNode], y[startNode]]

                        print "start index in ASTAR map:", pos_index_small
                        print "pos corresponding to start index in ASTAR map:", map_index_2_pos(map_msg, MAP_RES_ASTAR, pos_index_small)
                        print "value of start node in ASTAR map:", map_matrix_small[pos_index_small[1], pos_index_small[0]]

                    raw_path = astar_func([goal_pos_index_small[1], goal_pos_index_small[0]],
                                [pos_index_small[1], pos_index_small[0]], np.copy(map_matrix_small))

                    if raw_path is not None:
                        self.path = raw_path[1]

                        print "raw_path[0]:"
                        print raw_path[0]
                        print "raw_path[1]:"
                        print raw_path[1]

                        path = raw_path_2_path(raw_path[0], map_msg, MAP_RES_ASTAR)
                        print "computed path in get_path:", path
                    else:
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "Astar returned None, go to safe node!"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        pos_index_small = pos_2_map_index(map_msg, MAP_RES_ASTAR, pos)
                        temp = np.nonzero(map_matrix_small == 0)
                        x = temp[1]
                        y = temp[0]
                        distances = (x-pos_index_small[0])**2 + (y-pos_index_small[1])**2
                        goalNode = np.argmin(distances[np.where(distances > 1)])
                        goal_pos_index_small = [x[goalNode], y[goalNode]]

                        goal_pos = map_index_2_pos(map_msg, MAP_RES_ASTAR, goal_pos_index_small)
                        path = [goal_pos[0], goal_pos[1]]

                        self.path = np.array([[goal_pos_index_small[1]],[goal_pos_index_small[0]]])

                        print "goal_pos_index_small:", goal_pos_index_small
                        print "goal_pos:", goal_pos

                    print "end of get_path()"
                    print "****************************************************"

                    return path


            elif self.mode==2:
                print "COVERING MODE ENTERED"
                # while True:
                #     print "frontier done!"
                alpha=1.5
                obstacleMap2 = np.copy(self.map_covering)

                pos_index_mattias = pos_2_map_index(map_msg, MAP_RES_COVERING, pos)

                goalNodeCov = find_goal(obstacleMap2, [pos_index_mattias[1], pos_index_mattias[0]])
                #SCALING av startnider till ''
                #goalNodeCov = pos_2_map_index(map_msg, MAP_RES_COVERING, [0,0])
                coverPath, raw_coverPath = coverageMap(np.copy(map_matrix_small), obstacleMap2, alpha, [pos_index_mattias[1], pos_index_mattias[0]], goalNodeCov, map_msg)
                print "COVERING MODE finishED"
                print "coverPath:"
                print coverPath
                print "raw_coverPath:"
                print raw_coverPath
                path = raw_path_2_path(coverPath, map_msg, MAP_RES_ASTAR)
                self.path = raw_coverPath
                #print coverPath
                return path

        else:
            return None


    def check_path(self):
        rate = rospy.Rate(1) # (1 Hz)
        while not rospy.is_shutdown():
            if self.path is not None and self.map_small is not None:
                map_matrix = self.map_small
                raw_path = self.path

                raw_path = list(raw_path)
                map_path = map_matrix[raw_path[0], raw_path[1]]

                if 100 in map_path:
                    print "###############################################################################"
                    print "###############################################################################"
                    print "###############################################################################"
                    print "###############################################################################"
                    print "current path is NOT OK!"
                    print "###############################################################################"
                    print "###############################################################################"
                    print "###############################################################################"
                    print "###############################################################################"
                    msg_string = "stop"
                    self.warning_pub.publish(msg_string)

                    print "map_path in check_path:", map_path

                    path = self.get_path()
                    if path is not None:
                        print path
                        # publish the path:
                        msg = Float64MultiArray()
                        msg.data = path
                        self.path_pub.publish(msg)
                    else:
                        print "path is None!"
                else:
                    test = 1
                    # print "######################################"
                    # print "current path is OK!"
                    # print "######################################"

            rate.sleep() # (to get it to loop with 1 Hz)

if __name__ == "__main__":
    # create an Mapping object (this will run its __init__ function):
    mapping = Mapping()
    mapping.check_path()
