#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

import numpy as np
from operator import xor

import cv2
from scipy.ndimage.measurements import label

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

def raw_path_2_path(raw_path, map_msg):
    rows = raw_path[0].tolist()
    cols = raw_path[1].tolist()

    path = []

    for (row, col) in zip(rows, cols):
        pos_index = [col, row]
        pos = map_index_2_pos(map_msg, pos_index)
        path += [pos[0]]
        path += [pos[1]]

    return path

def astar_func(goalNode, startNode, obstacleMap):
    print "start of Astar!"

    slamMap = np.copy(obstacleMap)
    rows, cols = slamMap.shape

    # map_height, map_width = obstacleMap.shape
    # # convert map_visited to an image and save to disk (for visualization):
    # img = np.zeros((map_height, map_width, 3))
    # for row in range(map_height):
    #     for col in range(map_width):
    #         point = obstacleMap[row][col]
    #         if point == -1:
    #             img[row][col] = [100, 100, 100]
    #         elif point == 0:
    #             img[row][col] = [255, 255, 255]
    #         elif point == 100:
    #             img[row][col] = [0, 0, 0]
    # cv2.imwrite("test_astar.png", img)

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
    obstacleMap = slamMap

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

                xRoute=[]
                yRoute=[]
                xRoute.append(goalNode[0])
                yRoute.append(goalNode[1])

                dir=0
                while walkingNode[0]!=startNode[0] or walkingNode[1]!=startNode[1]:
                    numberOfsteps=numberOfsteps+1
                    prevX=int(fromXNode[walkingNode[0],walkingNode[1]])
                    prevY=int(fromYNode[walkingNode[0],walkingNode[1]])

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

                route=[xRoute,yRoute]
                routeS2G=np.fliplr(route)

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
                cv2.imwrite("test_astar_route.png", img)

                return routeS2G
                break

        print "end of Astar, currentNode"
        return currentNode
    else:
        return "Astar does NOT work!"

class Astar:
    def __init__(self):
        # initialize this code as a ROS node named nav_frontier_node:
        rospy.init_node("nav_astar_node", anonymous=True)

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        rospy.Subscriber("/estimated_pose", Float64MultiArray, self.est_pose_callback)

        # subscribe to the topic onto which the coordinator publishes data once
        # it has reached a goal position:
        rospy.Subscriber("/coordinator_status", String, self.coordinator_callback)

        # create a publisher for publishing goal positions to the controller:
        self.path_pub = rospy.Publisher("/path", Float64MultiArray, queue_size=10)

        self.map_msg = None
        self.x = None
        self.y = None
        self.theta = None

        self.goal_pos = [3, 1]

        msg = Float64MultiArray()
        msg.data = [0, 0.5]
        self.path_pub.publish(msg)

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

    # define the callback function for the /coordinator_status subscriber:
    def coordinator_callback(self, msg_obj):
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

        print "coordinator_callback"

    def get_path(self):
        if self.map_msg is not None and self.x is not None and self.y is not None:
            map_msg = self.map_msg
            x = self.x
            y = self.y
            pos = [x, y]

            map_matrix = map_msg_2_matrix(map_msg)
            #print map_matrix
            #print map_matrix.shape

            pos_index = pos_2_map_index(map_msg, pos)
            #print pos
            #print pos_index
            #print map_index_2_pos(map_msg, pos_index)

            goal_index = pos_2_map_index(map_msg, self.goal_pos)
            #print "goal:"
            #print self.goal_pos
            #print goal_index

            #print map_matrix[goal_index[1], goal_index[0]]

            raw_path = astar_func([goal_index[1], goal_index[0]], [pos_index[1], pos_index[0]], map_matrix)
            print raw_path
            path = raw_path_2_path(raw_path, map_msg)
            print path

            return path
        else:
            return None

if __name__ == "__main__":
    # create an Astar object (this will run its __init__ function):
    astar = Astar()
