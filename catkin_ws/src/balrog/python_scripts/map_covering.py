#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

from nav_msgs.msg import OccupancyGrid

from visualization_msgs.msg import Marker

import tf
import numpy as np

import cv2

import math

X_MAX = 4
X_MIN = -4
Y_MAX = 4
Y_MIN = -4

# -1 okant
# 0 inte hinder
# 100 hinder
# -2 visited

pub4 = rospy.Publisher("/map_covering", OccupancyGrid, queue_size=10)
pub3 = rospy.Publisher("/map_covering_large", OccupancyGrid, queue_size=10)
pub = rospy.Publisher("/map_covering_noedge_noenlarge", OccupancyGrid, queue_size=10)

most_recent_map = np.zeros(())
most_recent_map_origin = []
map_resolution = 0.05
most_recent_map_width = 0
most_recent_map_height = 0

pub_visitied = rospy.Publisher("/map_visited_local", OccupancyGrid, queue_size=10)

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
            # Width: 0.05 * COVERED_TILE_WIDTH [m]
            COVERED_TILE_WIDTH = 10
            for row in range(y_map_ind-COVERED_TILE_WIDTH, y_map_ind+COVERED_TILE_WIDTH + 1):
                for col in range(x_map_ind-COVERED_TILE_WIDTH, x_map_ind+COVERED_TILE_WIDTH + 1):
                    if row < map_height_local and col < map_width_local and row > 0 and col > 0:
                        map_point = map_local[row][col]
                        if map_point == 0:
                            img[row][col] = [0, 255, 0]
                            # Write to new map
                            map_local[row][col] = -2


            print "x: %f, y: %f" % (x, y)
            print "origin: (%f, %f)" % (map_origin_local[0], map_origin_local[1])
            print "x_map: %f, y_map: %f" % (x_map, y_map)
            print "x_map_ind: %d, y_map_ind: %d" % (x_map_ind, y_map_ind)
            print "##############################################"

        cv2.imwrite("BP5_map_visited.png", img)

        map_msg = OccupancyGrid()
        temp_msg = map_local.flatten()

        map_msg.info.height = map_height_local
        map_msg.info.width = map_width_local
        map_msg.info.resolution = 0.05
        map_msg.data = temp_msg.tolist()

        pub_visitied.publish(map_msg)

        # #########################################
        # ### ADD MAP COVERING HERE ###############
        # #########################################

        map_width = map_width_local
        map_height = map_height_local
        map_matrix = map_local

        # Number of cells to make box of. Side length = NR_OF_CELLS * 5cm
        # Size of box = NR_OF_CELLS^2
        NR_OF_CELLS = 3
        SAVE_IMAGE = True
        # Filter by counting content and looking for content in centre
        USE_FILTER = True
        MIN_NR_OF_OBSTACLES = 1
        MIN_NR_OF_NONVISITED = 2

        map_covering_height = int(math.floor(map_height / NR_OF_CELLS))
        map_covering_width = int(math.floor(map_width / NR_OF_CELLS))
        print "Visited map height: %f width: %f" % (map_height, map_width)
        print "Covering map height: %f width: %f" % (map_covering_height, map_covering_width)

        map_covering_img = np.zeros((map_covering_height, map_covering_width, 3))
        map_covering = np.zeros((map_covering_height, map_covering_width))

        #loop in every box
        for box_row in range(0, map_covering_height):
            for box_col in range(0 , map_covering_width):

                #loop for every cell to be part of box
                firstrow = box_row * NR_OF_CELLS
                firstcol = box_col * NR_OF_CELLS
                current_cell = -1
                nr_of_obstacles = 0
                nr_of_nonvisited = 0

                for row in range(firstrow, firstrow + NR_OF_CELLS):
                    for col in range(firstcol , firstcol + NR_OF_CELLS):
                        point = map_matrix[row][col]

                        # Obstacle in current_cell, mark obstacle always
                        if point == 100:
                            # Filter to check for more than MIN_NR_OF_OBSTACLES nodes with obstacle
                            if USE_FILTER == True and nr_of_obstacles <= MIN_NR_OF_OBSTACLES:
                                nr_of_obstacles += 1
                            else:
                                current_cell = 100

                        # current cell already visited and no obstacle
                        elif current_cell != 100 and point == -2:
                            # Check cells only in the middle of the box if filter = true
                            if USE_FILTER == True and row in range(firstrow + 1, firstrow + 3) and col in range(firstcol + 1, firstcol + 3):
                                current_cell = -2
                            elif USE_FILTER == False:
                                current_cell = -2

                        # current cell not visited and no obstacle
                        elif current_cell != 100 and current_cell != -2 and point == 0:
                            if USE_FILTER == True and nr_of_nonvisited <= MIN_NR_OF_NONVISITED:
                                nr_of_nonvisited += 1
                            else:
                                current_cell = 0
                        #else
                            #not explored = do nothing, current_cell = -1 is default

                # Write to covered_tiles_map
                map_covering[box_row][box_col] = current_cell
                # Write to image file
                if current_cell == -1:
                    map_covering_img[box_row][box_col] = [100, 100, 100]
                elif current_cell == 100:
                    map_covering_img[box_row][box_col] = [0, 0, 0]
                elif current_cell == -2:
                    map_covering_img[box_row][box_col] = [0, 255, 0]
                elif current_cell == 0:
                    map_covering_img[box_row][box_col] = [255, 255, 255]



        map_covering3_height = map_covering_height
        map_covering3_width = map_covering_width
        map_covering3 = np.copy(map_covering)
        map_covering3_img = np.zeros((map_covering3_height, map_covering3_width, 3))

        # If cells at the border are removed in small map. Add row
        if map_covering_height * NR_OF_CELLS < map_height:
            map_covering3_height += 1

            map_covering3 = np.zeros((map_covering3_height, map_covering3_width))
            map_covering3_img = np.zeros((map_covering3_height, map_covering3_width, 3))

            map_covering3[0:-1,:] = map_covering
            map_covering3_img[0:-1,:,:] = map_covering_img

            box_row = map_covering_height
            print "Extra box row: %f" % box_row
            for box_col in range(0 , map_covering3_width):
                current_cell = -1
                firstcol = box_col * NR_OF_CELLS

                for row in range(map_covering_height * NR_OF_CELLS, map_height):
                    for col in range(firstcol , firstcol + NR_OF_CELLS):
                        point = map_matrix[row][col]
                        # Obstacle in current_cell, mark obstacle always
                        if point == 100:
                            current_cell = 100
                        elif current_cell != 100 and point == -2:
                            current_cell = -2
                        elif current_cell != 100 and current_cell != -2 and point == 0:
                            current_cell = 0

                #print "Extra box row current_cell: %f" % current_cell
                # Write to covered_tiles_map
                map_covering3[box_row][box_col] = current_cell
                # Write to image file
                if current_cell == -1:
                    map_covering3_img[box_row][box_col] = [100, 100, 100]
                elif current_cell == 100:
                    map_covering3_img[box_row][box_col] = [0, 0, 0]
                elif current_cell == -2:
                    map_covering3_img[box_row][box_col] = [0, 255, 0]
                elif current_cell == 0:
                    map_covering3_img[box_row][box_col] = [255, 255, 255]


        #Add extra column
        if map_covering_width * NR_OF_CELLS < map_width:
            map_covering3_width += 1

            if map_covering_height * NR_OF_CELLS < map_height:
                # If extra row added, cannot copy directly to itself?
                map_covering_temp_img = np.zeros((map_covering3_height, map_covering3_width, 3))
                map_covering_temp = np.zeros((map_covering3_height, map_covering3_width))
                map_covering_temp[:,0:-1] = map_covering3
                map_covering_temp_img[:,0:-1,:] = map_covering3_img

                map_covering3_img = np.zeros((map_covering3_height, map_covering3_width, 3))
                map_covering3 = np.zeros((map_covering3_height, map_covering3_width))
                map_covering3 = map_covering_temp
                map_covering3_img = map_covering_temp_img
            else:
                map_covering3_img = np.zeros((map_covering3_height, map_covering3_width, 3))
                map_covering3 = np.zeros((map_covering3_height, map_covering3_width))
                map_covering3[:,0:-1] = map_covering
                map_covering3_img[:,0:-1,:] = map_covering_img

            box_col = map_covering_width
            print "Extra box col: %f" % box_col
            for box_row in range(0 , map_covering_height):
                current_cell = -1
                firstrow = box_row * NR_OF_CELLS

                print "new box"
                for row in range(firstrow , firstrow + NR_OF_CELLS):
                    for col in range(map_covering_width * NR_OF_CELLS, map_width):
                        print "extra col row: %f col: %f" % (row, col)
                        point = map_matrix[row][col]
                        # Obstacle in current_cell, mark obstacle always
                        if point == 100:
                            current_cell = 100
                        elif current_cell != 100 and point == -2:
                            current_cell = -2
                        elif current_cell != 100 and current_cell != -2 and point == 0:
                            current_cell = 0


                #print "Extra box col current_cell: %f" % current_cell
                # Write to covered_tiles_map
                map_covering3[box_row][box_col] = current_cell
                # Write to image file
                if current_cell == -1:
                    map_covering3_img[box_row][box_col] = [100, 100, 100]
                elif current_cell == 100:
                    map_covering3_img[box_row][box_col] = [0, 0, 0]
                elif current_cell == -2:
                    map_covering3_img[box_row][box_col] = [0, 255, 0]
                elif current_cell == 0:
                    map_covering3_img[box_row][box_col] = [255, 255, 255]



        # Obstacle EXPAND:
        OBSTACLE_EXPAND_SIZE = 2
        obst_inds = np.nonzero(map_covering3 == 100)
        obst_inds_row = obst_inds[0].tolist()
        obst_inds_col = obst_inds[1].tolist()
        obst_inds = zip(obst_inds_row, obst_inds_col)
        for obst_ind in obst_inds:
            obst_row = obst_ind[0]
            obst_col = obst_ind[1]
            for row in range(obst_row-OBSTACLE_EXPAND_SIZE, obst_row+OBSTACLE_EXPAND_SIZE+1):
                for col in range(obst_col-OBSTACLE_EXPAND_SIZE, obst_col+OBSTACLE_EXPAND_SIZE+1):
                    if row > 0 and row < map_covering_height and col > 0 and col < map_covering_width:
                        if map_covering[row][col] != 100:
                            map_covering3[row][col] = 70
                            map_covering3_img[row][col] = [0, 0, 250]

        # Save image file
        cv2.imwrite("BP5_map_covering1.png", map_covering_img)
        cv2.imwrite("BP5_map_covering3.png", map_covering3_img)

        # Publish covering tiles map as OccupancyGrid
        map_msg = OccupancyGrid()
        map_msg.info.resolution = 0.05 * NR_OF_CELLS
        map_msg.info.origin.position.x = map_origin_local[0]
        map_msg.info.origin.position.y = map_origin_local[1]

        map_msg.info.height = map_covering_height
        map_msg.info.width = map_covering_width
        map_msg_data = map_covering.flatten()
        map_msg.data = map_msg_data.tolist()
        pub.publish(map_msg)

        # Publish map with edges and enlarge
        map_msg.info.height = map_covering3_height
        map_msg.info.width = map_covering3_width
        map_msg_data = map_covering3.flatten()
        map_msg.data = map_msg_data.tolist()
        pub3.publish(map_msg)

        #################################################
        ############## final covering map ###############
        #################################################
        map_final_covering_height = int(math.floor(map_covering3_height / 2))
        map_final_covering_width = int(math.floor(map_covering3_width / 2))
        print "Final covering map height: %f width: %f" % (map_final_covering_height, map_final_covering_width)


        map_final_covering = np.zeros((map_final_covering_height, map_final_covering_width))

        for box_row in range(0, map_final_covering_height):
            for box_col in range(0 , map_final_covering_width):

                #loop for every cell to be part of box
                firstrow = box_row * 2
                firstcol = box_col * 2
                current_cell = -1

                for row in range(firstrow, firstrow + 2):
                    for col in range(firstcol , firstcol + 2):
                        point = map_covering3[row][col]
                        #if = 70 (expanded) hur hantera????

                        # obstacle in any cell
                        if point == 100:
                            current_cell = 100
                        elif current_cell != 100 and point == 70:
                            current_cell = 70
                        #not visited, no obstacle - mark to visit
                        elif current_cell != 100 and current_cell != 70 and point == 0:
                            current_cell = 0
                        # visited, no obstacle and no cell already found empty
                        elif current_cell != 100 and current_cell != 0 and current_cell != 70 and point == -2:
                            current_cell = -2

                # Write to covered_tiles_map
                map_final_covering[box_row][box_col] = current_cell



        if map_covering3_width%2 == 1 and map_covering3_height%2 == 1:
            print "Extra column and row needed"
            map_final_covering_width += 1
            map_final_covering_height += 1
            map_final_covering2 = np.zeros((map_final_covering_height, map_final_covering_width))
            map_final_covering2[0:-1,0:-1] = np.copy(map_final_covering)

            for row in range(0, map_final_covering_height-2):
                point = map_covering3[row*2][map_covering3_width-1]
                map_final_covering2[row][map_final_covering_width-1] = point

            for col in range(0, map_final_covering_width -2):
                point = map_covering3[map_covering3_height-1][col*2]
                map_final_covering2[map_final_covering_height-1][col] = point

        elif map_covering3_width%2 == 1:
            print "Extra column needed"
            map_final_covering_width += 1
            map_final_covering2 = np.zeros((map_final_covering_height, map_final_covering_width))
            map_final_covering2[:,0:-1] = np.copy(map_final_covering)

            for row in range(0, map_final_covering_height -2 ):
                point = map_covering3[row*2][map_covering3_width-1]
                map_final_covering2[row][map_final_covering_width-1] = point

        elif map_covering3_height%2 == 1:
            print "Extra row needed"
            map_final_covering_height += 1
            map_final_covering2 = np.zeros((map_final_covering_height, map_final_covering_width))
            map_final_covering2[0:-1,:] = np.copy(map_final_covering)

            for col in range(0, map_final_covering_width -2):
                point = map_covering3[map_covering3_height-1][col*2]
                map_final_covering2[map_final_covering_height-1][col] = point

        else:
            print "no extra row or column"
            map_final_covering2 = np.copy(map_final_covering)

        # publish final map
        map_msg.info.resolution = 0.05 * NR_OF_CELLS * 2
        map_msg.info.height = map_final_covering_height
        map_msg.info.width = map_final_covering_width
        temp_msg = map_final_covering2.flatten()
        map_msg.data = temp_msg.tolist()
        pub4.publish(map_msg)
        # np.savetxt('test.out', map_final_covering2, delimiter=',')

def callback_func_map(msg_obj):
    global most_recent_map, most_recent_map_origin, most_recent_map_width
    global most_recent_map_height

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

    print "Map from slam width: %d, height: %d" % (map_width, map_height)

    # convert the map to an image and save to disk (for visualization):
    img = np.zeros((map_height, map_width, 3))
    for row in range(map_height):
        for col in range(map_width):
            point = map_matrix[row][col]
            if point == -1:
                img[row][col] = [100, 100, 100]
            elif point == 0:
                img[row][col] = [255, 255, 255]
            elif point == 100:
                img[row][col] = [0, 0, 0]
    cv2.imwrite("BP5_map_from_slam.png", img)

# def callback_func2(msg_obj):
#     map_data = msg_obj.data
#     map_width = msg_obj.info.width
#     map_height = msg_obj.info.height
#     map_origin = msg_obj.info.origin.position
#     map_origin = [map_origin.x, map_origin.y]
#
#     # convert the map data into a numpy array:
#     map_matrix = np.array(map_data)
#     # split the array into a list of map_height arrays (a list containing each row):
#     map_matrix = np.split(map_matrix, map_height)
#     # convert the list of array into a 2D array:
#     map_matrix = np.array(map_matrix)
#
#     map_matrix_rows, map_matrix_cols = map_matrix.shape
#
#     # # filter lone obstacle points:
#     n_threshold = 2
#     slamMap_binary = np.zeros((map_matrix_rows, map_matrix_cols))
#     slamMap_binary[map_matrix == 100] = 1
#     labeled_array, num_features = label(slamMap_binary)
#     binc = np.bincount(labeled_array.ravel())
#     noise_idx = np.where(binc <= n_threshold)
#     shp = map_matrix.shape
#     mask = np.in1d(labeled_array, noise_idx).reshape(shp)
#     map_matrix[mask] = 0
#
#     # Obstacle EXPAND:
#     temp = np.copy(map_matrix)
#     OBSTACLE_EXPAND_SIZE = 11
#     obst_inds = np.nonzero(map_matrix == 100)
#     obst_inds_row = obst_inds[0].tolist()
#     obst_inds_col = obst_inds[1].tolist()
#     obst_inds = zip(obst_inds_row, obst_inds_col)
#     for obst_ind in obst_inds:
#         obst_row = obst_ind[0]
#         obst_col = obst_ind[1]
#         for row in range(obst_row-OBSTACLE_EXPAND_SIZE, obst_row+OBSTACLE_EXPAND_SIZE+1):
#             for col in range(obst_col-OBSTACLE_EXPAND_SIZE, obst_col+OBSTACLE_EXPAND_SIZE+1):
#                 if row >= 0 and row < map_matrix_rows and col >= 0 and col < map_matrix_cols:
#                     if temp[row][col] != 100:
#                         map_matrix[row][col] = 100
#
#     # set all nodes outside of the 8x8 square to 100 (= obstacle):
#     x_min = X_MIN
#     x_max = X_MAX
#     y_min = Y_MIN
#     y_max = Y_MAX
#     #
#     map_msg = msg_obj
#     temp = pos_2_map_index(map_msg, [x_min, y_min])
#     x_min_ind = temp[0]
#     y_min_ind = temp[1]
#     temp = pos_2_map_index(map_msg, [x_max, y_max])
#     x_max_ind = temp[0]
#     y_max_ind = temp[1]
#     #
#     if x_max_ind < map_matrix_cols:
#         map_matrix[:, x_max_ind:] = 100
#     if x_min_ind > 0:
#         map_matrix[:, 0:x_min_ind] = 100
#     if y_max_ind < map_matrix_rows:
#         map_matrix[y_max_ind:, :] = 100
#     if y_min_ind > 0:
#         map_matrix[0:y_min_ind, :] = 100
#
#     map_matrix[map_matrix==100]=-2
#
#     map_height = map_matrix_rows
#     map_width = map_matrix_cols
#     # Number of cells to make box of. Size of box = NR_OF_CELLS^2
#     NR_OF_CELLS = 20
#     MIN_NR_OF_OBSTACLES = 44
#     map_small_height = int(map_height / NR_OF_CELLS)
#     map_small_width = int(map_width / NR_OF_CELLS)
#     #print "Visited map height: %f width: %f" % (map_height, map_width)
#     #print "Covering map height: %f width: %f" % (map_small_height, map_small_width)
#     map_small = np.zeros((map_small_height, map_small_width))
#     #loop in every box
#     for box_row in range(0, map_small_height):
#         for box_col in range(0 , map_small_width):
#             #loop for every cell to be part of box
#             firstrow = box_row * NR_OF_CELLS
#             firstcol = box_col * NR_OF_CELLS
#             current_cell = 0
#             nr_of_obstacles = 0
#             for row in range(firstrow, firstrow + NR_OF_CELLS):
#                 for col in range(firstcol , firstcol + NR_OF_CELLS):
#                     point = map_matrix_expand[row][col]
#                     # Obstacle in current_cell, mark obstacle always
#                     if point == -2:
#                         # Filter to check for more than MIN_NR_OF_OBSTACLES nodes with obstacle
#                         if nr_of_obstacles <= MIN_NR_OF_OBSTACLES:
#                             nr_of_obstacles += 1
#                         else:
#                             current_cell = -2
#             # Write to covered_tiles_map
#             map_small[box_row][box_col] = current_cell
#
#     map_msg = OccupancyGrid()
#     map_msg.info.resolution = 0.05*NR_OF_CELLS
#     map_msg.info.height = map_small_height
#     map_msg.info.width = map_small_width
#     temp_msg = map_small.flatten()
#     map_msg.data = temp_msg.tolist()
#     pub4.publish(map_msg)



if __name__ == "__main__":
    # initialize this code as a ROS node named slam_visited:
    rospy.init_node("slam_visited", anonymous=True)

    # subscribe to the topic /encoder_data. That is, read every message (of type
    # Float64MultiArray) that is published on /encoder. We will do this by
    # calling the function callback_func every time a new message is published:
    rospy.Subscriber("/Mapper/vertices", Marker, callback_func)
    #rospy.Subscriber("/map_visited", OccupancyGrid, callback_func2)

    rospy.Subscriber("/map", OccupancyGrid, callback_func_map)

    # keep python from exiting until this ROS node is stopped:
    rospy.spin()
