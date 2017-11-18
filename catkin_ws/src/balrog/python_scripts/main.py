#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

from geometry_msgs.msg import PoseStamped
import tf

import numpy as np
import cv2
from scipy.ndimage.measurements import label
import time
import threading

from utilities import map_index_2_pos, pos_2_map_index, raw_path_2_path, map_msg_2_matrix
from nav_frontier import frontier_func
from nav_astar import astar_func
from nav_covering import coverageMap, find_goal

# size of considered area:
X_MAX = 7.5 # (NOTE! if this value is modified one also needs to update it in nav_covering.py)
X_MIN = -0.2 # (NOTE! if this value is modified one also needs to update it in nav_covering.py)
Y_MAX = 7.5 # (NOTE! if this value is modified one also needs to update it in nav_covering.py)
Y_MIN = -0.2 # (NOTE! if this value is modified one also needs to update it in nav_covering.py)

# map resolutions:
MAP_RES_SLAM = 0.05
MAP_RES_FRONTIER = 0.05
MAP_RES_ASTAR = 0.25 # (NOTE! if this value is modified one also needs to update it in nav_covering.py)
MAP_RES_COVERING = 0.30 # (NOTE! if this value is modified one also needs to update it in nav_covering.py)

class Main:
    def __init__(self):
        self.lock = threading.Lock()

        # initialize this code as a ROS node named main_node:
        rospy.init_node("main_node", anonymous=True)

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        rospy.Subscriber("/estimated_pose", Float64MultiArray, self.est_pose_callback)

        # subscribe to the topic onto which the coordinator publishes data once
        # it has reached the end of a path:
        rospy.Subscriber("/coordinator_status", String, self.coordinator_callback)

        rospy.Subscriber("/map_covering", OccupancyGrid, self.map_covering_callback)

        # create a publisher for publishing paths to the coordinator:
        self.path_pub = rospy.Publisher("/path", Float64MultiArray, queue_size=10)

        self.warning_pub = rospy.Publisher("/node_status", String, queue_size=1)

        self.mode_pub = rospy.Publisher("/balrog_mode", String, queue_size=1)

        self.x = None
        self.y = None
        self.theta = None

        self.raw_path = None

        self.map_origin = None
        self.map_matrix_slam = None
        self.map_matrix_frontier = None
        self.map_matrix_astar = None
        self.map_matrix_covering = None

        self.mode = "MAPPING" # (MAPPING, COVERING or MISSION_FINISHED)

        self.trans_listener = tf.TransformListener()

        self.mine_locations = {} # dictionary that caches the most updated locations of each detected mine
        self.mine_offset = PoseStamped()
        self.mine_offset.pose.position.z = 0.7 # mines are located 70 cm in front of the AprilTag
        quat = tf.transformations.quaternion_from_euler(0., np.pi/2, np.pi/2)
        self.mine_offset.pose.orientation.x = quat[0]
        self.mine_offset.pose.orientation.y = quat[1]
        self.mine_offset.pose.orientation.z = quat[2]
        self.mine_offset.pose.orientation.w = quat[3]

        self.mines_disarmed = False

        # get things moving (seems like we need to wait a short moment for the
        # message to actually be published):
        msg = Float64MultiArray()
        msg.data = [-0.1, 0, 0.1, 0]
        time.sleep(0.5)
        self.path_pub.publish(msg)

    def update_mines(self):
        # update the position for all visable mines (tags with offset):
        for tag_number in range(9): # (0, 1, .., 8)
            try:
                self.mine_offset.header.frame_id = "/tag_{0}".format(tag_number)
                self.mine_locations[tag_number] = self.trans_listener.transformPose("/map", self.mine_offset)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        print "detected tags:", self.mine_locations.keys()
        print "no of detected tags: %d/%d" % (len(self.mine_locations), 4)

    # define the callback function for the /map subscriber:
    def map_callback(self, msg_obj):
        print "map_callback"
        map_origin = msg_obj.info.origin
        map_matrix_slam = map_msg_2_matrix(msg_obj)

        map_matrix_slam_rows, map_matrix_slam_cols = map_matrix_slam.shape
        map_matrix_frontier = np.copy(map_matrix_slam)

        # # save map_matrix_frontier as an image:
        # map_height, map_width = map_matrix_frontier.shape
        # img = np.zeros((map_height, map_width, 3))
        # for row in range(map_height):
        #     for col in range(map_width):
        #         point = map_matrix_frontier[row][col]
        #         if point == -1:
        #             img[row][col] = [100, 100, 100]
        #         elif point == 0:
        #             img[row][col] = [255, 255, 255]
        #         elif point == 100:
        #             img[row][col] = [0, 0, 0]
        # cv2.imwrite("map_matrix_frontier1.png", img)

        # filter lone obstacle points:
        n_threshold = 2
        slamMap_binary = np.zeros((map_matrix_slam_rows, map_matrix_slam_cols))
        slamMap_binary[map_matrix_slam == 100] = 1
        labeled_array, num_features = label(slamMap_binary)
        binc = np.bincount(labeled_array.ravel())
        noise_idx = np.where(binc <= n_threshold)
        shp = map_matrix_slam.shape
        mask = np.in1d(labeled_array, noise_idx).reshape(shp)
        map_matrix_frontier[mask] = 0

        # # save map_matrix_frontier as an image:
        # map_height, map_width = map_matrix_frontier.shape
        # img = np.zeros((map_height, map_width, 3))
        # for row in range(map_height):
        #     for col in range(map_width):
        #         point = map_matrix_frontier[row][col]
        #         if point == -1:
        #             img[row][col] = [100, 100, 100]
        #         elif point == 0:
        #             img[row][col] = [255, 255, 255]
        #         elif point == 100:
        #             img[row][col] = [0, 0, 0]
        # cv2.imwrite("map_matrix_frontier2.png", img)

        # expand all obstacles:
        temp = np.copy(map_matrix_frontier)
        OBSTACLE_EXPAND_SIZE = 11
        obst_inds = np.nonzero(map_matrix_frontier == 100)
        obst_inds_row = obst_inds[0].tolist()
        obst_inds_col = obst_inds[1].tolist()
        obst_inds = zip(obst_inds_row, obst_inds_col)
        for obst_ind in obst_inds:
            obst_row = obst_ind[0]
            obst_col = obst_ind[1]
            for row in range(obst_row-OBSTACLE_EXPAND_SIZE, obst_row+OBSTACLE_EXPAND_SIZE+1):
                for col in range(obst_col-OBSTACLE_EXPAND_SIZE, obst_col+OBSTACLE_EXPAND_SIZE+1):
                    if row >= 0 and row < map_matrix_slam_rows and col >= 0 and col < map_matrix_slam_cols:
                        if temp[row][col] != 100:
                            map_matrix_frontier[row][col] = 100

        # # save map_matrix_frontier as an image:
        # map_height, map_width = map_matrix_frontier.shape
        # img = np.zeros((map_height, map_width, 3))
        # for row in range(map_height):
        #     for col in range(map_width):
        #         point = map_matrix_frontier[row][col]
        #         if point == -1:
        #             img[row][col] = [100, 100, 100]
        #         elif point == 0:
        #             img[row][col] = [255, 255, 255]
        #         elif point == 100:
        #             img[row][col] = [0, 0, 0]
        # cv2.imwrite("map_matrix_frontier3.png", img)

        # set all nodes outside of the considered area to 100 (= obstacle):
        x_min = X_MIN
        x_max = X_MAX
        y_min = Y_MIN
        y_max = Y_MAX
        #
        temp = pos_2_map_index(map_origin, MAP_RES_SLAM, [x_min, y_min])
        x_min_ind = temp[0]
        y_min_ind = temp[1]
        temp = pos_2_map_index(map_origin, MAP_RES_SLAM, [x_max, y_max])
        x_max_ind = temp[0]
        y_max_ind = temp[1]
        #
        if x_max_ind < map_matrix_slam_cols:
            map_matrix_frontier[:, x_max_ind:] = 100
        if x_min_ind > 0:
            map_matrix_frontier[:, 0:x_min_ind] = 100
        if y_max_ind < map_matrix_slam_rows:
            map_matrix_frontier[y_max_ind:, :] = 100
        if y_min_ind > 0:
            map_matrix_frontier[0:y_min_ind, :] = 100

        # # save map_matrix_frontier as an image:
        # map_height, map_width = map_matrix_frontier.shape
        # img = np.zeros((map_height, map_width, 3))
        # for row in range(map_height):
        #     for col in range(map_width):
        #         point = map_matrix_frontier[row][col]
        #         if point == -1:
        #             img[row][col] = [100, 100, 100]
        #         elif point == 0:
        #             img[row][col] = [255, 255, 255]
        #         elif point == 100:
        #             img[row][col] = [0, 0, 0]
        # cv2.imwrite("map_matrix_frontier4.png", img)

        map_height = map_matrix_slam_rows
        map_width = map_matrix_slam_cols
        NR_OF_CELLS = 5 # (no of cells to make box of, size of box = NR_OF_CELLS^2)
        MIN_NR_OF_OBSTACLES = 12
        map_astar_height = int(map_height / NR_OF_CELLS)
        map_astar_width = int(map_width / NR_OF_CELLS)
        map_matrix_astar = np.zeros((map_astar_height, map_astar_width))
        for box_row in range(0, map_astar_height):
            for box_col in range(0 , map_astar_width):
                #loop for every cell to be part of box
                firstrow = box_row * NR_OF_CELLS
                firstcol = box_col * NR_OF_CELLS
                current_cell = 0
                nr_of_obstacles = 0
                for row in range(firstrow, firstrow + NR_OF_CELLS):
                    for col in range(firstcol , firstcol + NR_OF_CELLS):
                        point = map_matrix_frontier[row][col]
                        # Obstacle in current_cell, mark obstacle always
                        if point == 100:
                            # Filter to check for more than MIN_NR_OF_OBSTACLES nodes with obstacle
                            if nr_of_obstacles <= MIN_NR_OF_OBSTACLES:
                                nr_of_obstacles += 1
                            else:
                                current_cell = 100
                map_matrix_astar[box_row][box_col] = current_cell

        # map_height, map_width = map_matrix_astar.shape
        # img = np.zeros((map_height, map_width, 3))
        # for row in range(map_height):
        #     for col in range(map_width):
        #         point = map_matrix_astar[row][col]
        #         if point == -1:
        #             img[row][col] = [100, 100, 100]
        #         elif point == 0:
        #             img[row][col] = [255, 255, 255]
        #         elif point == 100:
        #             img[row][col] = [0, 0, 0]
        # cv2.imwrite("map_matrix_astar.png", img)

        self.lock.acquire()
        self.map_origin = map_origin
        self.map_matrix_slam = map_matrix_slam
        self.map_matrix_frontier = map_matrix_frontier
        self.map_matrix_astar = map_matrix_astar
        self.lock.release()

    # define the callback function for the /map_covering subscriber:
    def map_covering_callback(self, msg_obj):
        print "map_covering_callback"

        map_origin = msg_obj.info.origin
        map_matrix_covering = map_msg_2_matrix(msg_obj)

        self.lock.acquire()
        self.map_origin = map_origin
        self.map_matrix_covering = map_matrix_covering
        self.lock.release()

    # define the callback function for the /estimated_pose subscriber:
    def est_pose_callback(self, msg_obj):
        #print "est_pose_callback"

        pose = msg_obj.data

        self.lock.acquire()
        self.x = pose[0]
        self.y = pose[1]
        self.theta = pose[2]
        self.lock.release()

    # define the callback function for the /coordinator_status subscriber:
    def coordinator_callback(self, msg_obj):
        if self.mode != "MISSION_FINISHED":
            print "coordinator_callback"

            # get the recived string:
            msg_string = msg_obj.data

            if msg_string == "reached end of path":
                path = self.get_path()

                if path is not None:
                    print "coordinator_callback: path to publish:", path

                    # publish the path:
                    msg = Float64MultiArray()
                    msg.data = path
                    self.path_pub.publish(msg)
                else:
                    print "coordinator_callback: path is None!"

    def get_path(self):
        if self.map_origin is not None and self.x is not None and self.y is not None and self.map_matrix_frontier is not None and self.map_matrix_astar is not None:
            self.lock.acquire()
            map_origin = self.map_origin
            x = self.x
            y = self.y
            pos = [x, y]
            map_matrix_frontier = self.map_matrix_frontier
            map_matrix_astar = self.map_matrix_astar
            map_matrix_covering = self.map_matrix_covering
            self.lock.release()

            pos_index_frontier = pos_2_map_index(map_origin, MAP_RES_FRONTIER, pos)
            pos_index_astar = pos_2_map_index(map_origin, MAP_RES_ASTAR, pos)

            # print "get_path: pos:", pos
            # print "get_path: pos_index_frontier:", pos_index_frontier
            # print "get_path: pos_index_astar:", pos_index_astar

            ##
            if self.mode == "MAPPING":
                goal_pos_index_frontier = frontier_func(np.copy(map_matrix_frontier), pos_index_frontier)

                if goal_pos_index_frontier == [-1000, -1000]:
                    print "MAPPING mode is finished, entering COVERING mode!"
                    self.mode = "COVERING"
                    path = self.get_path()
                    return path
                else:
                    goal_pos = map_index_2_pos(map_origin, MAP_RES_FRONTIER, goal_pos_index_frontier)
                    # print "goal index in FRONTIER map:", goal_pos_index_frontier
                    # print "goal pos:", goal_pos
                    # print "value of goal node in FRONTIER map:", map_matrix_frontier[goal_pos_index_frontier[1], goal_pos_index_frontier[0]]

                    goal_pos_index_astar = pos_2_map_index(map_origin, MAP_RES_ASTAR, goal_pos)
                    # print "goal index in ASTAR map:", goal_pos_index_astar
                    # print "goal pos corresponding to goal index in ASTAR map:", map_index_2_pos(map_origin, MAP_RES_ASTAR, goal_pos_index_astar)
                    # print "value of goal node in ASTAR map:", map_matrix_astar[goal_pos_index_astar[1], goal_pos_index_astar[0]]

                    if map_matrix_astar[goal_pos_index_astar[1], goal_pos_index_astar[0]] == 100:
                        # print "########################################################"
                        # print "get_path(): GOAL node is not free, get the closest free node!"
                        # print "########################################################"
                        temp = np.nonzero(map_matrix_astar == 0)
                        x = temp[1]
                        y = temp[0]
                        distances = (x-goal_pos_index_astar[0])**2 + (y-goal_pos_index_astar[1])**2
                        goal_node = np.argmin(distances)
                        goal_pos_index_astar = [x[goal_node], y[goal_node]]
                        # print "goal index in ASTAR map:", goal_pos_index_astar
                        # print "goal pos corresponding to goal index in ASTAR map:", map_index_2_pos(map_origin, MAP_RES_ASTAR, goal_pos_index_astar)
                        # print "value of goal node in ASTAR map:", map_matrix_astar[goal_pos_index_astar[1], goal_pos_index_astar[0]]

                    if map_matrix_astar[pos_index_astar[1], pos_index_astar[0]] == 100:
                        # print "########################################################"
                        # print "get_path(): START node is not free, get the closest free node!"
                        # print "########################################################"
                        temp = np.nonzero(map_matrix_astar == 0)
                        x = temp[1]
                        y = temp[0]
                        distances = (x-pos_index_astar[0])**2 + (y-pos_index_astar[1])**2
                        start_node = np.argmin(distances)
                        pos_index_astar = [x[start_node], y[start_node]]
                        # print "start index in ASTAR map:", pos_index_astar
                        # print "pos corresponding to start index in ASTAR map:", map_index_2_pos(map_origin, MAP_RES_ASTAR, pos_index_astar)
                        # print "value of start node in ASTAR map:", map_matrix_astar[pos_index_astar[1], pos_index_astar[0]]

                    astar_paths = astar_func([goal_pos_index_astar[1], goal_pos_index_astar[0]],
                                [pos_index_astar[1], pos_index_astar[0]], np.copy(map_matrix_astar))

                    if astar_paths is not None:
                        self.raw_path = astar_paths[1]
                        # print "astar_paths[0]:"
                        # print astar_paths[0]
                        # print "astar_paths[1]:"
                        # print astar_paths[1]

                        path = raw_path_2_path(astar_paths[0], map_origin, MAP_RES_ASTAR)
                    else:
                        print "################################################"
                        print "################################################"
                        print "get_path: astar_func returned None, go to safe node!"
                        print "################################################"
                        print "################################################"

                        temp = np.nonzero(map_matrix_astar == 0)
                        x = temp[1]
                        y = temp[0]
                        distances = (x-pos_index_astar[0])**2 + (y-pos_index_astar[1])**2
                        goal_node = np.argmin(distances[np.where(distances > 1)])
                        goal_pos_index_astar = [x[goal_node], y[goal_node]]

                        goal_pos = map_index_2_pos(map_origin, MAP_RES_ASTAR, goal_pos_index_astar)
                        path = [goal_pos[0], goal_pos[1]]

                        self.raw_path = np.array([[goal_pos_index_astar[1]],[goal_pos_index_astar[0]]])

                        # print "goal_pos_index_astar:", goal_pos_index_astar
                        # print "goal_pos:", goal_pos

                    return path

            ##
            elif self.mode == "COVERING":
                alpha=0.9

                pos_index_covering = pos_2_map_index(map_origin, MAP_RES_COVERING, pos)

                goal_pos = [0, 0]
                goal_index_covering = pos_2_map_index(map_origin, MAP_RES_COVERING, goal_pos)

                covering_paths = coverageMap(np.copy(map_matrix_astar), np.copy(map_matrix_covering), alpha, [pos_index_covering[1], pos_index_covering[0]], [goal_index_covering[1], goal_index_covering[0]], map_origin)

                if covering_paths is not None:
                    # print "coverPath:", covering_paths[0]
                    # print "raw_coverPath:", covering_paths[1]

                    path = raw_path_2_path(covering_paths[0], map_origin, MAP_RES_ASTAR)
                    self.raw_path = covering_paths[1]
                else:
                    print "covering_paths is None"
                    print "COVERING mode is finished, entering DISARMING mode!"
                    self.mode = "DISARMING"
                    path = self.get_path()
                    return path

                return path

            ##
            elif self.mode == "DISARMING":
                if not self.mines_disarmed:
                    path = []

                    start_pos = pos
                    start_index_astar = pos_2_map_index(map_origin, MAP_RES_ASTAR, start_pos)
                    if map_matrix_astar[start_index_astar[1], start_index_astar[0]] == 100:
                        temp = np.nonzero(map_matrix_astar == 0)
                        x = temp[1]
                        y = temp[0]
                        distances = (x-start_index_astar[0])**2 + (y-start_index_astar[1])**2
                        start_node = np.argmin(distances)
                        start_index_astar = [x[start_node], y[start_node]]

                    goal_pos = [0, 0] # (start by going back to the start pos to get disarming gear)
                    goal_index_astar = pos_2_map_index(map_origin, MAP_RES_ASTAR, goal_pos)
                    if map_matrix_astar[goal_index_astar[1], goal_index_astar[0]] == 100:
                        temp = np.nonzero(map_matrix_astar == 0)
                        x = temp[1]
                        y = temp[0]
                        distances = (x-goal_index_astar[0])**2 + (y-goal_index_astar[1])**2
                        goal_node = np.argmin(distances)
                        goal_index_astar = [x[goal_node], y[goal_node]]

                    astar_paths = astar_func([goal_index_astar[1], goal_index_astar[0]],
                                [start_index_astar[1], start_index_astar[0]], np.copy(map_matrix_astar))
                    if astar_paths is not None:
                        path += raw_path_2_path(astar_paths[0], map_origin, MAP_RES_ASTAR)
                    else:
                        print "astar_paths is None!"

                    prev_goal_index_astar = goal_index_astar

                    for mine_id in self.mine_locations:
                        mine_obj = self.mine_locations[mine_id]
                        mine_pos = [mine_obj.pose.position.x, mine_obj.pose.position.y]

                        start_index_astar = prev_goal_index_astar

                        goal_pos = mine_pos
                        goal_index_astar = pos_2_map_index(map_origin, MAP_RES_ASTAR, goal_pos)
                        if map_matrix_astar[goal_index_astar[1], goal_index_astar[0]] == 100:
                            temp = np.nonzero(map_matrix_astar == 0)
                            x = temp[1]
                            y = temp[0]
                            distances = (x-goal_index_astar[0])**2 + (y-goal_index_astar[1])**2
                            goal_node = np.argmin(distances)
                            goal_index_astar = [x[goal_node], y[goal_node]]

                        astar_paths = astar_func([goal_index_astar[1], goal_index_astar[0]],
                                    [start_index_astar[1], start_index_astar[0]], np.copy(map_matrix_astar))
                        if astar_paths is not None:
                            path += raw_path_2_path(astar_paths[0], map_origin, MAP_RES_ASTAR)
                        else:
                            print "astar_paths is None!"

                        prev_goal_index_astar = goal_index_astar

                    start_index_astar = prev_goal_index_astar

                    goal_pos = [0, 0] # (return to the starting point)
                    goal_index_astar = pos_2_map_index(map_origin, MAP_RES_ASTAR, goal_pos)
                    if map_matrix_astar[goal_index_astar[1], goal_index_astar[0]] == 100:
                        temp = np.nonzero(map_matrix_astar == 0)
                        x = temp[1]
                        y = temp[0]
                        distances = (x-goal_index_astar[0])**2 + (y-goal_index_astar[1])**2
                        goal_node = np.argmin(distances)
                        goal_index_astar = [x[goal_node], y[goal_node]]

                    astar_paths = astar_func([goal_index_astar[1], goal_index_astar[0]],
                                [start_index_astar[1], start_index_astar[0]], np.copy(map_matrix_astar))
                    if astar_paths is not None:
                        path += raw_path_2_path(astar_paths[0], map_origin, MAP_RES_ASTAR)
                    else:
                        print "astar_paths is None!"

                    self.mines_disarmed = True
                    return path
                else: # (if self.mines_disarmed:)
                    print "DISARMING mode is finished, entering MISSION_FINISHED mode!"
                    self.mode = "MISSION_FINISHED"
                    return None

        else:
            return None

    def check_path(self):
        if self.mode == "MAPPING":
            if self.raw_path is not None and self.map_matrix_astar is not None:
                self.lock.acquire()
                map_matrix_astar = self.map_matrix_astar
                raw_path = self.raw_path
                self.lock.release()

                raw_path = list(raw_path)
                map_path = map_matrix_astar[raw_path[0], raw_path[1]]

                if 100 in map_path:
                    msg_string = "stop"
                    self.warning_pub.publish(msg_string)

                    print "####################################################"
                    print "####################################################"
                    print "check_path: current path is NOT ok!"
                    print "####################################################"
                    print "####################################################"

                    print "check_path: map_path:", map_path

                    path = self.get_path()

                    if path is not None:
                        print "check_path: path to publish:", path

                        # publish the path:
                        msg = Float64MultiArray()
                        msg.data = path
                        self.path_pub.publish(msg)
                    else:
                        print "check_path: path is None!"

    def run(self):
        rate = rospy.Rate(2) # (2 Hz)

        while not rospy.is_shutdown():
            self.check_path()
            #self.update_mines()

            print "current mode: %s" % self.mode
            self.mode_pub.publish(self.mode)

            rate.sleep() # (to get it to loop with 2 Hz)

if __name__ == "__main__":
    # create a Main object (this will run its __init__ function):
    main = Main()

    # run the member function run:
    main.run()
