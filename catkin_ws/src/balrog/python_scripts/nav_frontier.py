import numpy as np
import cv2
from scipy.ndimage.measurements import label

# Frontier is a function which determines what node gets set as goal node.
# The definition of a frontier is an unexplored node which is adjecent to a an
# explored node. Here a 0 represents explored node and -1 represents an
# unexplored node. From that definition we can quantify the problem as to find
# every -1 which is next to a 0, those nodes are frontier nodes. Out of all
# these frontier nodes we choose the nearest frontier node (euclidean distance)
# as a goal node.

# By using overlap of the slamMap itself we can find the frontier. This
# however needs to be done twice in order to cover each corner and
# dircetion. The frontier as well as the nodes along the edges of the map
# are evaluated. This is done in two steps. slamMap is the name of the discretized
# map which is created by SLAM.

# STEP 1 (extending the map direction)
#                     -1         -1 -1 -1 -1 -1 -1 -1
#                     -1         -1
#                     -1         -1
#        slamMap      -1    -    -1      slamMap
#                     -1         -1
#                     -1         -1
#   -1 -1 -1 -1 -1 -1 -1         -1

# STEP 2 (extending the map direction)
#   -1 -1 -1 -1 -1 -1 -1         -1
#                     -1         -1
#                     -1         -1
#        slamMap      -1    -    -1       slamMap
#                     -1         -1
#                     -1         -1
#                     -1         -1 -1 -1 -1 -1 -1 -1

def frontier_func(slamMap, currentPosition):
    rows, cols = slamMap.shape
    frontierMap = np.zeros(shape=(rows,cols))
    #print slamMap

    # debugging:
    # # save slamMap as an image:
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
    # cv2.imwrite("frontierMap1.png", img)

    slamMapNoObstacles = np.copy(slamMap) # create a copy of map
    slamMapNoObstacles[slamMapNoObstacles == 100] = -1 # remove any obstacles and define them as unexplored

    tempMap1 = -1*np.ones((rows+1, cols+1)) # expanding map with -1
    tempMap1[1:, 1:] = slamMapNoObstacles   # importing the map
    tempMap2 = -1*np.ones((rows+1, cols+1)) # expanding map with -1
    tempMap2[0:-1, 0:-1] = slamMapNoObstacles # importing the map

    # As shown above (STEP 1)
    frontierNodes = tempMap2 - tempMap1 ## 16x16

    FirstMap = frontierNodes[0:-1,0:-1] # scale down the map
    SecondMap = frontierNodes[1:,1:]    # scale down the map

    frontierMap[FirstMap == -1] = 1 # -1 in FirstMap represent frontier nodes
    frontierMap[SecondMap == 1] = 1 # 1 in SecondMap represent frontier nodes

    tempMap3 = -1*np.ones((rows+1, cols+1)) # expanding map with -1
    tempMap3[0:-1, 1:] = slamMapNoObstacles # importing the map
    tempMap4 = -1*np.ones((rows+1, cols+1)) # expanding map with -1
    tempMap4[1:, 0:-1] = slamMapNoObstacles # importing the map

    # As shown above (STEP 2)
    frontierNodes = tempMap3 - tempMap4

    FirstMap = frontierNodes[:-1,1:]    # scale down the map
    SecondMap = frontierNodes[1:,0:-1]  # scale down the map

    frontierMap[FirstMap == -1] = 1 # -1 in FirstMap represent frontier nodes
    frontierMap[SecondMap == 1] = 1 # 1 in SecondMap represent frontier nodes

    # debugging:
    # # create image of frontier map
    # map_height, map_width = frontierMap.shape
    # img = np.zeros((map_height, map_width, 3))
    # for row in range(map_height):
    #     for col in range(map_width):
    #         point = frontierMap[row][col]
    #         if point == 1:
    #             img[row][col] = [255, 0, 0]
    #         elif point == 0:
    #             img[row][col] = [255, 255, 255]
    # cv2.imwrite("frontierMap2.png", img)

    # Remove obstacles as frontier candidates:
    frontierMap[slamMap == 100] = 0

    # debugging:
    # # create image of frontier map
    # map_height, map_width = frontierMap.shape
    # img = np.zeros((map_height, map_width, 3))
    # for row in range(map_height):
    #     for col in range(map_width):
    #         point = frontierMap[row][col]
    #         if point == 1:
    #             img[row][col] = [255, 0, 0]
    #         elif point == 0:
    #             img[row][col] = [255, 255, 255]
    # cv2.imwrite("frontierMap3.png", img)

    # # filter lone frontier nodes: (n_threshold is the boundary of the number of
    # # connected nodes which gets filtrated). The filter is optional.
    # n_threshold = 1
    # labeled_array, num_features = label(frontierMap)
    # binc = np.bincount(labeled_array.ravel())
    # noise_idx = np.where(binc <= n_threshold)
    # shp = frontierMap.shape
    # mask = np.in1d(labeled_array, noise_idx).reshape(shp)
    # frontierMap[mask] = 0
    # #print frontierMap

    # debugging:
    # # create image of frontier map
    # map_height, map_width = frontierMap.shape
    # img = np.zeros((map_height, map_width, 3))
    # for row in range(map_height):
    #     for col in range(map_width):
    #         point = frontierMap[row][col]
    #         if point == 1:
    #             img[row][col] = [255, 0, 0]
    #         elif point == 0:
    #             img[row][col] = [255, 255, 255]
    # cv2.imwrite("frontierMap4.png", img)

    # remove nodes that are too close to the current node: (this can be tuned in order
    # to achieve a smoother behavior.
    for row in range(currentPosition[1]-30, currentPosition[1]+31):
        for col in range(currentPosition[0]-30, currentPosition[0]+31):
            if row < rows and col < cols:
                frontierMap[row][col] = 0

    # debugging:
    # # create image of frontier map
    # map_height, map_width = frontierMap.shape
    # img = np.zeros((map_height, map_width, 3))
    # for row in range(map_height):
    #     for col in range(map_width):
    #         point = frontierMap[row][col]
    #         if point == 1:
    #             img[row][col] = [255, 0, 0]
    #         elif point == 0:
    #             img[row][col] = [255, 255, 255]
    # cv2.imwrite("frontierMap5.png", img)

    # frontierMap consists of only 1's and 0's. Where the 1's represent a frontier.
    temp = np.nonzero(frontierMap) # fetch all frontier candidates
    x = temp[1] # set x as x (since currentPosition = [x,y])
    y = temp[0] # set y as y

    # set condition for when the frontier is done, since frontier map is just a matrix of
    # zeros where the ones represent a frontier. Therefore the number of ones in frontier map
    # represent the amount of frontiers.
    if len(x) > 0:
        # return the frontier node which is closest to currentPosition
        goalNode = np.argmin((x-currentPosition[0])**2 + (y-currentPosition[1])**2)
        return [x[goalNode], y[goalNode]] # ([x (col), y (row)])
    else:
        print "frontier is done!"
        return [-1000, -1000] # return a predefined node as a msg
