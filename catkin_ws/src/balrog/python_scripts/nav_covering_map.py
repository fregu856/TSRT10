#!/usr/bin/env python

# TODO! insert general comment on what the code does here

import rospy
from nav_msgs.msg import OccupancyGrid

import numpy as np
import cv2
import math

# -2 visited node
# -1 unknown node
# 0 free node
# 100 obstacle node

class NavCoveringMap:
    def __init__(self):
        # initialize this code as a ROS node named nav_covering_map_node:
        rospy.init_node("nav_covering_map_node", anonymous=True)

        # subscribe to the map_visted topic:
        rospy.Subscriber("/map_visited", OccupancyGrid, self.callback_func)

        # create publisher for publishing first step of covering map:
        self.debug_pub = rospy.Publisher("/map_covering_large", OccupancyGrid, queue_size=10)

        # create publisher for publishing the final covering map:
        self.pub = rospy.Publisher("/map_covering", OccupancyGrid, queue_size=10)

        # keep python from exiting until this ROS node is stopped:
        rospy.spin()

    # callback function for the /map_visited topic:
    def callback_func(self, msg_obj):
            print ""
            print "####### New map_covering ########"

            # Receive map_visited as an OccupancyGrid object
            map_data = msg_obj.data
            map_width = msg_obj.info.width
            map_height = msg_obj.info.height
            map_origin = msg_obj.info.origin.position
            map_origin = [map_origin.x, map_origin.y]

            # convert the map data into a numpy array:
            map_visited = np.array(map_data)
            # split the array into a list of map_height arrays (a list containing each row):
            map_visited = np.split(map_visited, map_height)
            # convert the list of array into a 2D array:
            map_visited = np.array(map_visited)

            #####################################################
            ############ First section of cell merging ##########
            #####################################################

            # Number of cells to merge. Side length = NR_OF_CELLS * 5cm
            # New merged cell is called box

            # Size of box = NR_OF_CELLS^2
            # If NR_OF_CELLS = 3, 9 cells are merged into 1

            # ###
            # ### -> #
            # ###

            NR_OF_CELLS = 3

            # Filter by counting content and looking for content in centre
            USE_FILTER = True
            MIN_NR_OF_OBSTACLES = 2
            MIN_NR_OF_NONVISITED = 3

            # Dimensions of new map
            # Rounded down do ensure index within bounds
            map_covering_height = int(math.floor(map_height / NR_OF_CELLS))
            map_covering_width = int(math.floor(map_width / NR_OF_CELLS))

            # Map matrix created for both OccupancyGrid on topic and image export
            map_covering = np.zeros((map_covering_height, map_covering_width))
            map_covering_img = np.zeros((map_covering_height, map_covering_width, 3))
            print "Visited map height: %f width: %f" % (map_height, map_width)

            ##########################################
            # Main loop for all cells in new map (box)
            ##########################################
            for box_row in range(0, map_covering_height):
                for box_col in range(0 , map_covering_width):

                    #######################################
                    # Loop for every cell to be part of box
                    #######################################
                    # Priority order:
                    # Obstacle
                    # Visited
                    # Not visited
                    # Not explored

                    firstrow = box_row * NR_OF_CELLS
                    firstcol = box_col * NR_OF_CELLS
                    current_box = -1
                    nr_of_obstacles = 0
                    nr_of_nonvisited = 0

                    # All cells are searched and decides what the box shall be
                    # with the current_box-variable
                    for row in range(firstrow, firstrow + NR_OF_CELLS):
                        for col in range(firstcol , firstcol + NR_OF_CELLS):
                            current_cell = map_visited[row][col]

                            # Highest priority: Obstacle in current cell
                            if current_cell == 100:
                                # Filter to check for more than MIN_NR_OF_OBSTACLES nodes with obstacle
                                if USE_FILTER == True and nr_of_obstacles < MIN_NR_OF_OBSTACLES:
                                    nr_of_obstacles += 1
                                else:
                                    current_box = 100

                            # Current cell visited and no obstacle found
                            elif current_box != 100 and current_cell == -2:
                                # # Check cells only in the middle of the box if filter = true
                                # if USE_FILTER == True and row in range(firstrow + 1, firstrow + 2) and col in range(firstcol + 1, firstcol + 2):
                                #     current_box = -2
                                # elif USE_FILTER == False:
                                current_box = -2

                            # current cell not visited and no obstacle found
                            elif current_box != 100 and current_box != -2 and current_cell == 0:
                                # Filter to check for more than MIN_NR_OF_NONVISITED nodes that are not visited
                                if USE_FILTER == True and nr_of_nonvisited < MIN_NR_OF_NONVISITED:
                                    nr_of_nonvisited += 1
                                else:
                                    current_box = 0
                            #else
                                #not explored = do nothing, current_box = -1 is default

                    # Write the resulting value of current_box to both matrix and image
                    map_covering[box_row][box_col] = current_box
                    if current_box == -1:
                        map_covering_img[box_row][box_col] = [100, 100, 100]
                    elif current_box == 100:
                        map_covering_img[box_row][box_col] = [0, 0, 0]
                    elif current_box == -2:
                        map_covering_img[box_row][box_col] = [0, 255, 0]
                    elif current_box == 0:
                        map_covering_img[box_row][box_col] = [255, 255, 255]

            #####################################################################
            # When dimension of map_covering are rounded down, data might be lost
            # Following section will add extra row or column if necessary
            #####################################################################
            map_covering2_height = map_covering_height
            map_covering2_width = map_covering_width
            map_covering2 = np.copy(map_covering)
            map_covering2_img = np.zeros((map_covering2_height, map_covering2_width, 3))


            # Extra row check
            if map_covering_height * NR_OF_CELLS < map_height:
                map_covering2_height += 1

                # Empty row created in a new matrix, map_covering2
                map_covering2 = -1*np.ones((map_covering2_height, map_covering2_width))
                map_covering2_img = -1*np.ones((map_covering2_height, map_covering2_width, 3))
                map_covering2[0:-1,:] = np.copy(map_covering)
                map_covering2_img[0:-1,:,:] = np.copy(map_covering_img)

                box_row = map_covering_height
                print "Extra row added: %f" % box_row
                for box_col in range(0 , map_covering2_width):
                    current_box = -1
                    firstcol = box_col * NR_OF_CELLS

                    for row in range(map_covering_height * NR_OF_CELLS, map_height):
                        for col in range(firstcol , firstcol + NR_OF_CELLS):
                            current_cell = map_visited[row][col]
                            # Obstacle in box, set obstacle always
                            if current_cell == 100:
                                current_box = 100
                            elif current_box != 100 and current_cell == -2:
                                current_box = -2
                            elif current_box != 100 and current_box != -2 and current_cell == 0:
                                current_box = 0

                    #print "Extra box row current_box: %f" % current_box
                    map_covering2[box_row][box_col] = current_box
                    if current_box == -1:
                        map_covering2_img[box_row][box_col] = [100, 100, 100]
                    elif current_box == 100:
                        map_covering2_img[box_row][box_col] = [0, 0, 0]
                    elif current_box == -2:
                        map_covering2_img[box_row][box_col] = [0, 255, 0]
                    elif current_box == 0:
                        map_covering2_img[box_row][box_col] = [255, 255, 255]

            # Extra column check
            if map_covering_width * NR_OF_CELLS < map_width:
                map_covering2_width += 1

                if map_covering_height * NR_OF_CELLS < map_height:
                    # If extra row added, cannot copy directly to itself
                    map_covering_temp_img = -1*np.ones((map_covering2_height, map_covering2_width, 3))
                    map_covering_temp = -1*np.ones((map_covering2_height, map_covering2_width))
                    map_covering_temp[:,0:-1] = np.copy(map_covering2)
                    map_covering_temp_img[:,0:-1,:] = np.copy(map_covering2_img)

                    map_covering2_img = -1*np.ones((map_covering2_height, map_covering2_width, 3))
                    map_covering2 = -1*np.ones((map_covering2_height, map_covering2_width))
                    map_covering2 = np.copy(map_covering_temp)
                    map_covering2_img = np.copy(map_covering_temp_img)
                else:
                    map_covering2_img = -1*np.ones((map_covering2_height, map_covering2_width, 3))
                    map_covering2 = -1*np.ones((map_covering2_height, map_covering2_width))
                    map_covering2[:,0:-1] = np.copy(map_covering)
                    map_covering2_img[:,0:-1,:] = np.copy(map_covering_img)

                box_col = map_covering_width
                print "Extra column added: %f" % box_col
                for box_row in range(0 , map_covering_height):
                    current_box = -1
                    firstrow = box_row * NR_OF_CELLS

                    for row in range(firstrow , firstrow + NR_OF_CELLS):
                        for col in range(map_covering_width * NR_OF_CELLS, map_width):
                            current_cell = map_visited[row][col]
                            # Obstacle in box, set obstacle always
                            if current_cell == 100:
                                current_box = 100
                            elif current_box != 100 and current_cell == -2:
                                current_box = -2
                            elif current_box != 100 and current_box != -2 and current_cell == 0:
                                current_box = 0

                    #print "Extra box col current_box: %f" % current_box

                    map_covering2[box_row][box_col] = current_box
                    if current_box == -1:
                        map_covering2_img[box_row][box_col] = [100, 100, 100]
                    elif current_box == 100:
                        map_covering2_img[box_row][box_col] = [0, 0, 0]
                    elif current_box == -2:
                        map_covering2_img[box_row][box_col] = [0, 255, 0]
                    elif current_box == 0:
                        map_covering2_img[box_row][box_col] = [255, 255, 255]

            ###################################################################
            # Expand obstacles
            # Coordinte system in centre of Balrog and cannot visit cells close
            # to objects
            ###################################################################
            OBSTACLE_EXPAND_SIZE = 2

            obst_inds = np.nonzero(map_covering2 == 100)
            obst_inds_row = obst_inds[0].tolist()
            obst_inds_col = obst_inds[1].tolist()
            obst_inds = zip(obst_inds_row, obst_inds_col)
            for obst_ind in obst_inds:
                obst_row = obst_ind[0]
                obst_col = obst_ind[1]
                for row in range(obst_row-OBSTACLE_EXPAND_SIZE, obst_row+OBSTACLE_EXPAND_SIZE+1):
                    for col in range(obst_col-OBSTACLE_EXPAND_SIZE, obst_col+OBSTACLE_EXPAND_SIZE+1):
                        # To avoid cells outside the matrix
                        if row >= 0 and row < map_covering_height and col >= 0 and col < map_covering_width:
                            # If no obstacle in cell: Set to 70 to indicate expanded
                            if map_covering[row][col] != 100:
                                map_covering2[row][col] = 70
                                map_covering2_img[row][col] = [0, 0, 250]

            print "Covering map height: %f width: %f" % (map_covering2_height, map_covering2_width)

            # Save image files. In home folder?
            cv2.imwrite("BP5_map_covering1.png", map_covering_img)
            cv2.imwrite("BP5_map_covering2.png", map_covering2_img)

            # Publish map_coverint_large as OccupancyGrid
            map_msg = OccupancyGrid()
            map_msg.info.resolution = 0.05 * NR_OF_CELLS
            map_msg.info.origin.position.x = map_origin[0]
            map_msg.info.origin.position.y = map_origin[1]

            map_msg.info.height = map_covering2_height
            map_msg.info.width = map_covering2_width
            map_msg_data = map_covering2.flatten()
            map_msg.data = map_msg_data.tolist()
            self.debug_pub.publish(map_msg)

            ##############################################
            ###### Second section of cell merging ########
            ##############################################

            # Reduction of the grid size by 2
            # 4 cells are merged into 1

            ## -> #
            ##

            map_final_covering_height = int(math.floor(map_covering2_height / 2))
            map_final_covering_width = int(math.floor(map_covering2_width / 2))
            map_final_covering = np.zeros((map_final_covering_height, map_final_covering_width))

            for box_row in range(0, map_final_covering_height):
                for box_col in range(0 , map_final_covering_width):

                    #loop for every cell to be part of box
                    firstrow = box_row * 2
                    firstcol = box_col * 2
                    current_box = -1

                    for row in range(firstrow, firstrow + 2):
                        for col in range(firstcol , firstcol + 2):
                            current_cell = map_covering2[row][col]

                            # obstacle
                            if current_cell == 100:
                                current_box = 100
                            # expanded obstacle
                            elif current_box != 100 and current_cell == 70:
                                current_box = 70
                            #not visited, no obstacle
                            elif current_box != 100 and current_box != 70 and current_cell == 0:
                                current_box = 0
                            # visited, no obstacle and no cell empty
                            elif current_box != 100 and current_box != 0 and current_box != 70 and current_cell == -2:
                                current_box = -2

                    # Save in matrix
                    map_final_covering[box_row][box_col] = current_box

            ####################################################################
            # If height and/or width is odd number, data is removed when merging
            # Extra row/column are needed
            ####################################################################

            if map_covering2_width%2 == 1 and map_covering2_height%2 == 1:
                print "Extra column and row added"
                map_final_covering_width += 1
                map_final_covering_height += 1
                map_final_covering2 = -1*np.ones((map_final_covering_height, map_final_covering_width))
                map_final_covering2[0:-1,0:-1] = np.copy(map_final_covering)

                for row in range(0, map_final_covering_height):
                    current_cell = map_covering2[row*2][map_covering2_width-1]
                    map_final_covering2[row][map_final_covering_width-1] = current_cell

                for col in range(0, map_final_covering_width):
                    current_cell = map_covering2[map_covering2_height-1][col*2]
                    map_final_covering2[map_final_covering_height-1][col] = current_cell

            elif map_covering2_width%2 == 1:
                print "Extra column added"
                map_final_covering_width += 1
                map_final_covering2 = -1*np.ones((map_final_covering_height, map_final_covering_width))
                map_final_covering2[:,0:-1] = np.copy(map_final_covering)

                for row in range(0, map_final_covering_height ):
                    current_cell = map_covering2[row*2][map_covering2_width-1]
                    map_final_covering2[row][map_final_covering_width-1] = current_cell

            elif map_covering2_height%2 == 1:
                print "Extra row added"
                map_final_covering_height += 1
                map_final_covering2 = -1*np.ones((map_final_covering_height, map_final_covering_width))
                map_final_covering2[0:-1,:] = np.copy(map_final_covering)

                for col in range(0, map_final_covering_width):
                    current_cell = map_covering2[map_covering2_height-1][col*2]
                    map_final_covering2[map_final_covering_height-1][col] = current_cell

            else:
                #print "no extra row or column"
                map_final_covering2 = np.copy(map_final_covering)

            print "Final covering map height: %f width: %f" % (map_final_covering_height, map_final_covering_width)

            # publish final map
            map_msg = OccupancyGrid()
            map_msg.info.origin.position.x = map_origin[0]
            map_msg.info.origin.position.y = map_origin[1]
            map_msg.info.resolution = 0.05 * NR_OF_CELLS * 2
            map_msg.info.height = map_final_covering_height
            map_msg.info.width = map_final_covering_width

            temp_msg = map_final_covering2.flatten()
            map_msg.data = temp_msg.tolist()
            self.pub.publish(map_msg)

            # np.savetxt('test.out', map_final_covering2, delimiter=',')

if __name__ == "__main__":
    # create a NavCoveringMap object (this will run its __init__ function):
    nav_covering_map = NavCoveringMap()
