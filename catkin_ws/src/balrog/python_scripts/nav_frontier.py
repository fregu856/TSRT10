import numpy as np
import cv2
from scipy.ndimage.measurements import label

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
    cv2.imwrite("frontierMap1.png", img)

    slamMapNoObstacles = np.copy(slamMap)
    slamMapNoObstacles[slamMapNoObstacles == 100] = -1   #Obstacle]
    #print slamMapNoObstacles

    tempMap1 = -1*np.ones((rows+1, cols+1))
    tempMap1[1:, 1:] = slamMapNoObstacles
    #print tempMap1

    tempMap2 = -1*np.ones((rows+1, cols+1))
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

    tempMap3 = -1*np.ones((rows+1, cols+1))
    tempMap3[0:-1, 1:] = slamMapNoObstacles
    #print tempMap3

    tempMap4 = -1*np.ones((rows+1, cols+1))
    tempMap4[1:, 0:-1] = slamMapNoObstacles
    #print tempMap4

    frontierNodes = tempMap3 - tempMap4
    #print frontierNodes

    FirstMap = frontierNodes[:-1,1:]
    SecondMap = frontierNodes[1:,0:-1]
    frontierMap[FirstMap == -1] = 1
    frontierMap[SecondMap == 1] = 1
    #print frontierMap

    map_height, map_width = frontierMap.shape
    img = np.zeros((map_height, map_width, 3))
    for row in range(map_height):
        for col in range(map_width):
            point = frontierMap[row][col]
            if point == 1:
                img[row][col] = [255, 0, 0]
            elif point == 0:
                img[row][col] = [255, 255, 255]
    cv2.imwrite("frontierMap2.png", img)

    # Remove obstacles as frontier candidates:
    frontierMap[slamMap == 100] = 0

    map_height, map_width = frontierMap.shape
    img = np.zeros((map_height, map_width, 3))
    for row in range(map_height):
        for col in range(map_width):
            point = frontierMap[row][col]
            if point == 1:
                img[row][col] = [255, 0, 0]
            elif point == 0:
                img[row][col] = [255, 255, 255]
    cv2.imwrite("frontierMap3.png", img)

    # # filter lone frontier nodes:
    # n_threshold = 1
    # labeled_array, num_features = label(frontierMap)
    # binc = np.bincount(labeled_array.ravel())
    # noise_idx = np.where(binc <= n_threshold)
    # shp = frontierMap.shape
    # mask = np.in1d(labeled_array, noise_idx).reshape(shp)
    # frontierMap[mask] = 0
    # #print frontierMap

    map_height, map_width = frontierMap.shape
    img = np.zeros((map_height, map_width, 3))
    for row in range(map_height):
        for col in range(map_width):
            point = frontierMap[row][col]
            if point == 1:
                img[row][col] = [255, 0, 0]
            elif point == 0:
                img[row][col] = [255, 255, 255]
    cv2.imwrite("frontierMap4.png", img)

    # remove nodes that are too close to the current node:
    for row in range(currentPosition[1]-30, currentPosition[1]+31):
        for col in range(currentPosition[0]-30, currentPosition[0]+31):
            if row < rows and col < cols:
                frontierMap[row][col] = 0

    map_height, map_width = frontierMap.shape
    img = np.zeros((map_height, map_width, 3))
    for row in range(map_height):
        for col in range(map_width):
            point = frontierMap[row][col]
            if point == 1:
                img[row][col] = [255, 0, 0]
            elif point == 0:
                img[row][col] = [255, 255, 255]
    cv2.imwrite("frontierMap5.png", img)

    temp = np.nonzero(frontierMap)
    x = temp[1]
    y = temp[0]
    if len(x) > 0:
        goalNode = np.argmin((x-currentPosition[0])**2 + (y-currentPosition[1])**2)
        return [x[goalNode], y[goalNode]] # ([x (col), y (row)])
    else:
        print "frontier is done!"
        return [-1000, -1000]
