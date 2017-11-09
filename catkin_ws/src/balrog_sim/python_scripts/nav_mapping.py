#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

import numpy as np
from operator import xor

import cv2
from scipy.ndimage.measurements import label

import time

def map_index_2_pos(map_msg, pos_index): #mappa from index in map to real pos
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

def pos_2_map_index(map_msg, pos): # real pos to mapp index
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

def map_msg_2_matrix(map_msg): #
    map_data = map_msg.data
    map_height = map_msg.info.height

    # convert the map data from a list into a numpy array:
    map_matrix = np.array(map_data)
    # split the array into a list of map_height arrays (each list element is a row):
    map_matrix = np.split(map_matrix, map_height)
    # convert the list of arrays into a 2D array:
    map_matrix = np.array(map_matrix)

    return map_matrix

def raw_path_2_path(raw_path, map_msg):
    rows = raw_path[0].tolist() ######################################################################## before: rows = raw_path[1].tolist()
    cols = raw_path[1].tolist() ######################################################################## before: cols = raw_path[0].tolist()
    path = []

    for (row, col) in zip(rows, cols):
        pos_index = [col, row]
        pos = map_index_2_pos(map_msg, pos_index)
        path += [pos[0]]
        path += [pos[1]]

    return path

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
    # obst_inds = np.nonzero(slamMap == 100)
    # obst_inds_row = obst_inds[0].tolist()
    # obst_inds_col = obst_inds[1].tolist()
    # obst_inds = zip(obst_inds_row, obst_inds_col)
    # for obst_ind in obst_inds:
    #     obst_row = obst_ind[0]
    #     obst_col = obst_ind[1]
    #     for row in range(obst_row-6, obst_row+7):
    #         for col in range(obst_col-6, obst_col+7):
    #             if row < rows and col < cols:
    #                 slamMap[row][col] = 100

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

    slamMapNoObstacles = np.copy(slamMap)
    slamMapNoObstacles[slamMapNoObstacles == -2] = 0    #Covered
    slamMapNoObstacles[slamMapNoObstacles == 100] = -1   #Obstacle]
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

    map_height, map_width = frontierMap.shape
    img = np.zeros((map_height, map_width, 3))
    for row in range(map_height):
        for col in range(map_width):
            point = frontierMap[row][col]
            if point == 1:
                img[row][col] = [255, 0, 0]
            elif point == 0:
                img[row][col] = [255, 255, 255]
    cv2.imwrite("frontierMap1.png", img)

    # Remove covered area and obstacles as frontier candidates:
    frontierMap[slamMap == 100] = 0
    frontierMap[slamMap == -2] = 0

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
    cv2.imwrite("slamMap2.png", img)

    # onlyObs = slamMap
    # onlyObs[slamMap == -2] = 0
    #
    # clearFrontiers1_A = np.zeros((rows+1,cols))
    # clearFrontiers1_B = np.zeros((rows+1,cols))
    # clearFrontiers1_A[0:-1, :] = onlyObs
    # clearFrontiers1_B[1:, :] = onlyObs
    # clearFrontiers1 = clearFrontiers1_A - clearFrontiers1_B
    #
    # clearFrontiers2_A = np.zeros((rows,cols+1))
    # clearFrontiers2_B = np.zeros((rows,cols+1))
    # clearFrontiers2_A[:, 0:-1] = onlyObs
    # clearFrontiers2_B[:, 1:] = onlyObs
    # clearFrontiers2 = clearFrontiers2_A - clearFrontiers2_B
    #
    # frontierMap[clearFrontiers1[1:, :] == 101] = 0;
    # frontierMap[clearFrontiers1[1:, :] == 1] = 1;
    # frontierMap[clearFrontiers1[0:-1, :] == -101] = 0;
    # frontierMap[clearFrontiers1[0:-1, :] == -1] = 1;
    #
    # frontierMap[clearFrontiers2[:, 1:] == 101] = 0;
    # frontierMap[clearFrontiers2[:, 1:] == 1] = 1;
    # frontierMap[clearFrontiers2[:, 0:-1] == -101] = 0;
    # frontierMap[clearFrontiers2[:, 0:-1] == -1] = 1;


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

    # filter lone frontier nodes:
    n_threshold = 3
    labeled_array, num_features = label(frontierMap)
    binc = np.bincount(labeled_array.ravel())
    noise_idx = np.where(binc <= n_threshold)
    shp = frontierMap.shape
    mask = np.in1d(labeled_array, noise_idx).reshape(shp)
    frontierMap[mask] = 0
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
    cv2.imwrite("frontierMap4.png", img)

    # remove nodes that are too close to the current node:
    for row in range(currentPosition[1]-6, currentPosition[1]+7):
        for col in range(currentPosition[0]-6, currentPosition[0]+7):
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
        while True:
            print "frontier is done!"

def astar_func(goalNode, startNode, obstacleMap):
    print "start of Astar!"

    slamMap = np.copy(obstacleMap)
    rows, cols = slamMap.shape

    map_height, map_width = obstacleMap.shape
    # convert map_visited to an image and save to disk (for visualization):
    img = np.zeros((map_height, map_width, 3))
    for row in range(map_height):
        for col in range(map_width):
            point = obstacleMap[row][col]
            if point == -1:
                img[row][col] = [100, 100, 100]
            elif point == 0:
                img[row][col] = [255, 255, 255]
            elif point == 100:
                img[row][col] = [0, 0, 0]
    img[goalNode[0]][goalNode[1]] = [0, 255, 0]
    cv2.imwrite("test_astar.png", img)

    # # filter lone obstacle points:
    # n_threshold = 3
    # slamMap_binary = np.zeros((rows, cols))
    # slamMap_binary[slamMap == 100] = 1
    # labeled_array, num_features = label(slamMap_binary)
    # binc = np.bincount(labeled_array.ravel())
    # noise_idx = np.where(binc <= n_threshold)
    # shp = slamMap.shape
    # mask = np.in1d(labeled_array, noise_idx).reshape(shp)
    # slamMap[mask] = 0
    #
    # map_height, map_width = slamMap.shape
    # # convert map_visited to an image and save to disk (for visualization):
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
    # cv2.imwrite("test_astar_filtered.png", img)

    # expand all obstacles:
    # obst_inds = np.nonzero(slamMap == 100)
    # obst_inds_row = obst_inds[0].tolist()
    # obst_inds_col = obst_inds[1].tolist()
    # obst_inds = zip(obst_inds_row, obst_inds_col)
    # for obst_ind in obst_inds:
    #     obst_row = obst_ind[0]
    #     obst_col = obst_ind[1]
    #     for row in range(obst_row-6, obst_row+7):
    #         for col in range(obst_col-6, obst_col+7):
    #             if row < rows and col < cols:
    #                 slamMap[row][col] = 100
    # obstacleMap = slamMap

    # save slamMap as an image after obstacle expansion:
    map_height, map_width = obstacleMap.shape
    img = np.zeros((map_height, map_width, 3))
    for row in range(map_height):
        for col in range(map_width):
            point = obstacleMap[row][col]
            if point == -1:
                img[row][col] = [100, 100, 100]
            elif point == 0:
                img[row][col] = [255, 255, 255]
            elif point == 100:
                img[row][col] = [0, 0, 0]
    img[goalNode[0]][goalNode[1]] = [0, 255, 0]
    cv2.imwrite("test_astar_expanded.png", img)

    [xmax,ymax]=obstacleMap.shape
    fCostMap=np.full((xmax,ymax),float('nan'))
    fCostMapOld=np.full((xmax,ymax),float('nan'))
    hCostMap=np.zeros((xmax,ymax))
    gCostMap=np.full((xmax,ymax),float('nan'))
    gCostMapOld=np.full((xmax,ymax),float('nan'))
    checkedMap=np.zeros((xmax,ymax))
    fValueMap=np.zeros((xmax,ymax))
    fromXNode=np.zeros((xmax,ymax))
    fromYNode=np.zeros((xmax,ymax))
    found=0
    currentNode=startNode
    hDiffXMap=np.zeros(obstacleMap.shape)
    hDiffYMap=np.zeros(obstacleMap.shape)
    for xcounter in range(0,xmax):
        hDiffXMap[xcounter,:]=abs(xcounter-goalNode[0])
    for ycounter in range(0,ymax):
        hDiffYMap[:,ycounter]=abs(ycounter-goalNode[1])
    hCostMap=2**(0.5)*np.minimum(hDiffXMap,hDiffYMap)+np.maximum(hDiffXMap,hDiffYMap)-np.minimum(hDiffXMap,hDiffYMap)
    gCostMapOld[currentNode[0],currentNode[1]]=0

    sum=0
    if True:
        while 1:
            #print sum
            sum +=1
        #for p in range(0,2):
            checkedMap[currentNode[0], currentNode[1]]=1
            for i in range(-1,2):
                for j in range(-1,2):
                    temp=[]
                    temp1=[]
                    b=gCostMapOld[currentNode[0],currentNode[1]]

                    if (currentNode[0]+i)>-1 and (currentNode[1]+j)>-1 and (currentNode[0]+i)<xmax and (currentNode[1]+j)<ymax and xor(j==0, i==0)  and obstacleMap[(currentNode[0]+i),(currentNode[1]+j)]!=100:

                        gCostMap[currentNode[0]+i,(currentNode[1]+j)]=b+1
                        temp=gCostMapOld[currentNode[0]+i,(currentNode[1]+j)]
                        if (gCostMap[currentNode[0]+i,(currentNode[1]+j)]>temp):
                            gCostMap[currentNode[0]+i,(currentNode[1]+j)]=temp
                            bigger=1

                        else:
                            fromXNode[currentNode[0]+i,(currentNode[1]+j)]=currentNode[0]
                            fromYNode[currentNode[0]+i,(currentNode[1]+j)]=currentNode[1]
                            fValueMap[currentNode[0]+i,(currentNode[1]+j)]=1

                    elif (currentNode[0]+i)>-1 and (currentNode[1]+j)>-1 and (currentNode[0]+i)<xmax and (currentNode[1]+j)<ymax and i!=0 and j!=0  and obstacleMap[(currentNode[0]+i),(currentNode[1]+j)]!=100:
                        gCostMap[currentNode[0]+i,(currentNode[1]+j)]=b+2**(0.5)
                        temp1=gCostMapOld[currentNode[0]+i,(currentNode[1]+j)]

                        if (gCostMap[currentNode[0]+i,(currentNode[1]+j)]>temp1):
                            gCostMap[currentNode[0]+i,(currentNode[1]+j)]=temp1
                        else:
                            fromXNode[currentNode[0]+i,(currentNode[1]+j)]=currentNode[0]
                            fromYNode[currentNode[0]+i,(currentNode[1]+j)]=currentNode[1]
                            fValueMap[currentNode[0]+i,(currentNode[1]+j)]=1
            gCostMap[currentNode[0],currentNode[1]]=b
            gCostMap[startNode[0],startNode[1]]=0

            minRow=[]
            minColumn=[]
            minRowElim=[]
            minColumnElim=[]
            minHValue=[]
            lookH=[]
            minPosition=[]
            numMinPoints=[]
            fCostMap=np.add(gCostMap,hCostMap)
            searchFCost=fCostMap*fValueMap*(checkedMap!=1)*(obstacleMap!=100)
            if np.nansum(searchFCost)>0:
                minPosition=np.where(searchFCost==np.nanmin(searchFCost[np.nonzero(searchFCost)]))
            else:
                route=[-1000, -1000]
                print "route = [-1000, -1000] break!"
                break

            numMinPoints=np.shape(minPosition)
            minRow=minPosition[0]
            minColumn=minPosition[1]
            for nodeCounter in range(0,numMinPoints[1]):
                lookH.append(hCostMap[minRow[nodeCounter],minColumn[nodeCounter]])

            minHValue=np.where(lookH==np.min(lookH))

            currentNode=[minRow[minHValue[0][0]],minColumn[minHValue[0][0]]]

            gCostMapOld=np.copy(gCostMap)
            fCostMapOld=np.copy(fCostMap)


            if fValueMap[goalNode[0],goalNode[1]]==1:
                found=1
                walkingNode=goalNode
                numberOfsteps=0
                wholeXroute=[]
                wholeYroute=[]
                xRoute=[]
                yRoute=[]
                xRoute.append(goalNode[0])
                yRoute.append(goalNode[1])

                dir=0
                while walkingNode[0]!=startNode[0] or walkingNode[1]!=startNode[1]:
                    numberOfsteps=numberOfsteps+1
                    prevX=int(fromXNode[walkingNode[0],walkingNode[1]])
                    prevY=int(fromYNode[walkingNode[0],walkingNode[1]])
                    wholeXroute.append(prevX)
                    wholeYroute.append(prevY)

                    if prevX==walkingNode[0]:
                        if dir!=5:
                            xRoute.append((walkingNode[0]))
                            yRoute.append(walkingNode[1])
                            dir=5
                    elif prevY==walkingNode[1]:
                        if dir!=3:
                            xRoute.append((walkingNode[0]))
                            yRoute.append(walkingNode[1])
                            dir=3
                    elif (prevY==walkingNode[1]+1 and prevY==walkingNode[1]+1) or (prevY==walkingNode[1]-1 and prevY==walkingNode[1]-1):
                        if dir!=2:
                            xRoute.append((walkingNode[0]))
                            yRoute.append(walkingNode[1])
                            dir=2
                    else:
                        if dir!=-2:
                            xRoute.append((walkingNode[0]))
                            yRoute.append(walkingNode[1])
                            dir=-2

                    walkingNode=[prevX,prevY]
                    #wholeRoute = [wholeXroute, wholeYroute]
                route=[xRoute, yRoute] ############################################################################################################################### before: route=[xRoute, xRoute]
                routeS2G=np.fliplr(route)
                wholeRoute = np.fliplr([wholeXroute, wholeYroute])

                # save obstacleMap as an image, with each point on the A* path
                # marked in red:
                map_height, map_width = obstacleMap.shape
                img = np.zeros((map_height, map_width, 3))
                for row in range(map_height):
                    for col in range(map_width):
                        point = obstacleMap[row][col]
                        if point == -1:
                            img[row][col] = [100, 100, 100]
                        elif point == 0:
                            img[row][col] = [255, 255, 255]
                        elif point == 100:
                            img[row][col] = [0, 0, 0]
                cols = yRoute
                rows = xRoute
                for (row, col) in zip(rows, cols):
                    img[row, col] = [0, 0, 255]
                img[goalNode[0]][goalNode[1]] = [0, 255, 0]
                cv2.imwrite("test_astar_route.png", img)

                return routeS2G, wholeRoute
                break

        print "end of Astar, currentNode"
        return currentNode
    else:
        return "Astar does NOT work!"

class Mapping:
    def __init__(self):
        # initialize this code as a ROS node named nav_mapping_node:
        rospy.init_node("nav_mapping_node", anonymous=True)

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        rospy.Subscriber("/estimated_pose", Float64MultiArray, self.est_pose_callback)

        # subscribe to the topic onto which the coordinator publishes data once
        # it has reached the end of a path:
        rospy.Subscriber("/coordinator_status", String, self.coordinator_callback)

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

        # get things moving (seems like we need to wait a short moment for the
        # message to actually be published):a
        msg = Float64MultiArray()
        msg.data = [0, 0.5]
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

        # # filter lone obstacle points:
        n_threshold = 5
        slamMap_binary = np.zeros((map_matrix_rows, map_matrix_cols))
        slamMap_binary[map_matrix == 100] = 1
        labeled_array, num_features = label(slamMap_binary)
        binc = np.bincount(labeled_array.ravel())
        noise_idx = np.where(binc <= n_threshold)
        shp = map_matrix.shape
        mask = np.in1d(labeled_array, noise_idx).reshape(shp)
        map_matrix[mask] = 0

        # Obstacle EXPAND:
        OBSTACLE_EXPAND_SIZE = 6
        obst_inds = np.nonzero(map_matrix == 100)
        obst_inds_row = obst_inds[0].tolist()
        obst_inds_col = obst_inds[1].tolist()
        obst_inds = zip(obst_inds_row, obst_inds_col)
        for obst_ind in obst_inds:
            obst_row = obst_ind[0]
            obst_col = obst_ind[1]
            for row in range(obst_row-OBSTACLE_EXPAND_SIZE, obst_row+OBSTACLE_EXPAND_SIZE+1):
                for col in range(obst_col-OBSTACLE_EXPAND_SIZE, obst_col+OBSTACLE_EXPAND_SIZE+1):
                    if row >= 0 and row < map_matrix_rows and col >= 0 and col < map_matrix_cols:
                        if map_matrix[row][col] != 100:
                            map_matrix_expand[row][col] = 100

        self.map_matrix_expand = map_matrix_expand
        print "map_callback"

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
                print path
                # publish the path:
                msg = Float64MultiArray()
                msg.data = path
                self.path_pub.publish(msg)
            else:
                print "path is None!"

        print "end of coordinator_callback"


    def check_path(self):
        rate = rospy.Rate(1) # (1 Hz)
        while not rospy.is_shutdown():
            if self.path is not None and self.goal_pos_index is not None and self.map_msg is not None:
                map_matrix = self.map_matrix_expand
                raw_path = self.path
                map_msg = self.map_msg
                goal_pos_index = self.goal_pos_index
                x = self.x
                y = self.y

                rows, cols = map_matrix.shape

                print "pos:"
                pos = [x, y]
                print pos
                map_matrix = self.map_matrix_expand
                print "pos_index:"
                pos_index = pos_2_map_index(map_msg, pos)
                print pos_index
                print "************************************"
                print map_matrix[max(pos_index[1]-1, 0):min(pos_index[1]+2, rows), max(pos_index[0]-1, 0):min(pos_index[0]+2, cols)]
                print "************************************"
                #tempPath = [raw_path[1], raw_path[0]] # (x,y)
                #tempPath = list(tempPath)
                #map_path = map_matrix[tempPath[1], tempPath[0]] ################################################################### before: map_path = map_matrix[tempPath[0], tempPath[1]] (tror det ar ratt nu, men inte 100 saker)
                raw_path = list(raw_path)
                map_path = map_matrix[raw_path[0], raw_path[1]]

                print "map_path in check_path:"
                print map_path

                if map_matrix[goal_pos_index[1],goal_pos_index[0]] == 100: # if goal_pos is not allowed
                    print "###############################################################################"
                    print "###############################################################################"
                    print "###############################################################################"
                    print "###############################################################################"
                    print('goal_pos Not allowed')
                    print "###############################################################################"
                    print "###############################################################################"
                    print "###############################################################################"
                    print "###############################################################################"
                    msg_string = "stop"
                    self.warning_pub.publish(msg_string)
                    path = self.get_path()
                    if path is not None:
                        print path
                        # publish the path:
                        msg = Float64MultiArray()
                        msg.data = path
                        self.path_pub.publish(msg)
                    else:
                        print "path is None!"

                elif 100 in map_path:
                    print "###############################################################################"
                    print "###############################################################################"
                    print "###############################################################################"
                    print "###############################################################################"
                    print('Route Not allowed')
                    print "###############################################################################"
                    print "###############################################################################"
                    print "###############################################################################"
                    print "###############################################################################"
                    msg_string = "stop"
                    self.warning_pub.publish(msg_string)

                    if 0 in map_matrix[max(pos_index[1]-1, 0):min(pos_index[1]+2, rows), max(pos_index[0]-1, 0):min(pos_index[0]+2, cols)].flatten().tolist() or -1 in map_matrix[max(pos_index[1]-1, 0):min(pos_index[1]+2, rows), max(pos_index[0]-1, 0):min(pos_index[0]+2, cols)].flatten().tolist():
                        print "route not allowed: do astar!"
                        raw_path = astar_func([goal_pos_index[1], goal_pos_index[0]],
                                    [pos_index[1], pos_index[0]], np.copy(map_matrix))
                        self.path = raw_path[1]
                        path = raw_path_2_path(raw_path[0], map_msg)
                    else:
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "route not allowed: go to safe node!"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                        temp1 = map_matrix == 0
                        temp2 = map_matrix == -1
                        temp3 = np.nonzero(temp1 + temp2)
                        x = temp3[1]
                        y = temp3[0]
                        goalNode = np.argmin((x-pos_index[0])**2 + (y-pos_index[1])**2)
                        goal_pos_index = [x[goalNode], y[goalNode]]

                        goal_pos = map_index_2_pos(map_msg, goal_pos_index)
                        path = [goal_pos[0], goal_pos[1]]

                        self.path = np.array([[goal_pos_index[1]],[goal_pos_index[0]]])

                        print "goal_pos_index:"
                        print goal_pos_index
                        print "goal_pos:"
                        print goal_pos

                    if path is not None:
                        print path
                        # publish the path:
                        msg = Float64MultiArray()
                        msg.data = path
                        self.path_pub.publish(msg)
                    else:
                        print "path is None!"
                else:
                    print "######################################"
                    print "current path is OK!"
                    print "######################################"

            rate.sleep() # (to get it to loop with 1 Hz)

    def get_path(self):
        if self.map_msg is not None and self.x is not None and self.y is not None:
            map_msg = self.map_msg
            x = self.x
            y = self.y
            pos = [x, y]
            map_matrix = self.map_matrix_expand
            pos_index = pos_2_map_index(map_msg, pos)

            rows, cols = map_matrix.shape

            print "pos:"
            print pos
            print "pos index:"
            print pos_index
            print "************************************"
            print map_matrix[max(pos_index[1]-1, 0):min(pos_index[1]+2, rows), max(pos_index[0]-1, 0):min(pos_index[0]+2, cols)]
            print "************************************"
            #print map_index_2_pos(map_msg, pos_index)

            if 0 in map_matrix[max(pos_index[1]-1, 0):min(pos_index[1]+2, rows), max(pos_index[0]-1, 0):min(pos_index[0]+2, cols)].flatten().tolist() or -1 in map_matrix[max(pos_index[1]-1, 0):min(pos_index[1]+2, rows), max(pos_index[0]-1, 0):min(pos_index[0]+2, cols)].flatten().tolist():
                goal_pos_index = frontier_func(np.copy(map_matrix), pos_index, map_msg)
                self.goal_pos_index = goal_pos_index
                print "goal index:"
                print goal_pos_index
                print "goal pos:"
                print map_index_2_pos(map_msg, goal_pos_index)
                print "value of goal node:"
                print map_matrix[goal_pos_index[1], goal_pos_index[0]]

                raw_path = astar_func([goal_pos_index[1], goal_pos_index[0]],
                            [pos_index[1], pos_index[0]], np.copy(map_matrix))
                self.path = raw_path[1]

                print "raw_path[0]:" ########################################### raw_path[0] var fel! Fixat nu!
                print raw_path[0]
                print "raw_path[1]:"
                print raw_path[1]

                path = raw_path_2_path(raw_path[0], map_msg)
                print "computed path in get_path:"
                print path
            else:
                print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                print "go to safe node!"
                print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                print "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"
                temp1 = map_matrix == 0
                temp2 = map_matrix == -1
                temp3 = np.nonzero(temp1 + temp2)
                x = temp3[1]
                y = temp3[0]
                goalNode = np.argmin((x-pos_index[0])**2 + (y-pos_index[1])**2)
                goal_pos_index = [x[goalNode], y[goalNode]]

                goal_pos = map_index_2_pos(map_msg, goal_pos_index)
                path = [goal_pos[0], goal_pos[1]]

                self.path = np.array([[goal_pos_index[1]],[goal_pos_index[0]]])

                print "goal_pos_index:"
                print goal_pos_index
                print "goal_pos:"
                print goal_pos

            return path
        else:
            return None

if __name__ == "__main__":
    # create an Mapping object (this will run its __init__ function):
    mapping = Mapping()
    mapping.check_path()
