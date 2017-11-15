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

def test_map():
    print "test map Covering Tiles"



def map_index_2_pos(map_msg, pos_index): #mappa from index in map to real pos
    map_origin_obj = map_msg.info.origin
    map_origin = [map_origin_obj.position.x, map_origin_obj.position.y]

    map_resolution = map_msg.info.resolution

    x_map_ind = pos_index[0]
    y_map_ind = pos_index[1]

    x_map = x_map_ind*map_resolution
    y_map = y_map_ind*map_resolution

    x = x_map + map_origin[0]
    y = y_map + map_origin[1]

    pos = [x, y]

    return pos

def map_index_2_pos_small(map_msg, pos_index): #mappa from index in map to real pos
    map_origin_obj = map_msg.info.origin
    map_origin = [map_origin_obj.position.x, map_origin_obj.position.y]

    map_resolution = 0.25

    x_map_ind = pos_index[0]
    y_map_ind = pos_index[1]

    x_map = x_map_ind*map_resolution
    y_map = y_map_ind*map_resolution

    x = x_map + map_origin[0]
    y = y_map + map_origin[1]

    pos = [x, y]

    return pos

def map_index_2_pos_mattias(map_msg, pos_index): #mappa from index in map to real pos
    map_origin_obj = map_msg.info.origin
    map_origin = [map_origin_obj.position.x, map_origin_obj.position.y]

    map_resolution = 0.40

    x_map_ind = pos_index[0]
    y_map_ind = pos_index[1]

    x_map = x_map_ind*map_resolution
    y_map = y_map_ind*map_resolution

    x = x_map + map_origin[0]
    y = y_map + map_origin[1]

    pos = [x, y]

    return pos

def pos_2_map_index(map_msg, pos): # real pos to map index
    map_origin_obj = map_msg.info.origin
    map_origin = [map_origin_obj.position.x, map_origin_obj.position.y]

    map_resolution = map_msg.info.resolution

    x = pos[0]
    y = pos[1]

    x_map = x - map_origin[0]
    y_map = y - map_origin[1]

    x_map_ind = int(x_map/map_resolution) # (col)
    y_map_ind = int(y_map/map_resolution) # (row)

    pos_index = [x_map_ind, y_map_ind]

    return pos_index

def pos_2_map_index_small(map_msg, pos): # real pos to map index
    map_origin_obj = map_msg.info.origin
    map_origin = [map_origin_obj.position.x, map_origin_obj.position.y]

    map_resolution = 0.25

    x = pos[0]
    y = pos[1]

    x_map = x - map_origin[0]
    y_map = y - map_origin[1]

    x_map_ind = int(x_map/map_resolution) # (col)
    y_map_ind = int(y_map/map_resolution) # (row)

    pos_index = [x_map_ind, y_map_ind]

    return pos_index

def pos_2_map_index_mattias(map_msg, pos): # real pos to map index
    map_origin_obj = map_msg.info.origin
    map_origin = [map_origin_obj.position.x, map_origin_obj.position.y]

    map_resolution = 0.40

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

def raw_path_2_path_small(raw_path, map_msg):
    rows = raw_path[0].tolist() ######################################################################## before: rows = raw_path[1].tolist()
    cols = raw_path[1].tolist() ######################################################################## before: cols = raw_path[0].tolist()
    path = []

    for (row, col) in zip(rows, cols):
        pos_index = [col, row]
        pos = map_index_2_pos_small(map_msg, pos_index)
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
    cv2.imwrite("astar.png", img)

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
    for i in range(15000):
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
            print "np.nansum(searchFCost)>0 is NOT true, break!"
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
                elif (prevX==walkingNode[0]+1 and prevY==walkingNode[1]+1) or (prevX==walkingNode[0]-1 and prevY==walkingNode[1]-1):
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
            cv2.imwrite("astar_route.png", img)

            print "Astar found a route!"
            print "end of Astar!"
            return routeS2G, wholeRoute

    print "Astar didn't find a route in time!"
    print "end of Astar!"
    return None

#coveringMode

def clObMap(obstacleMap):
#Making map, showing closeness to obstacles
    size=obstacleMap.shape
    closeObst=np.full(size,-1 )
    closeObst[(obstacleMap==-900)]=0    #set obstacles to 0

    run=1
    cost=0

    while run:
        pos=np.where(closeObst==cost)
        rowPos=pos[0]
        colPos=pos[1]
        rowLength=rowPos.shape

        cost=cost+1
        if rowLength[0]>0:
            for i in range(0,rowLength[0]):
                for e in range (-1,2):
                    for f in range (-1,2):
                        if  (rowPos[i]+e)>=0 and (rowPos[i]+e)<size[0] and (colPos[i]+f)>=0 and (colPos[i]+f)<size[1] and closeObst[rowPos[i]+e,colPos[i]+f]==-1:
                            closeObst[rowPos[i]+e,colPos[i]+f]=cost
        else:
            run=0

    return np.nanmax(closeObst)-closeObst

def clGoMap(obstacleMap, goalNode):
    size=obstacleMap.shape
    closeGoal=np.full(size,-1)
    closeGoal[goalNode[0],goalNode[1]]=0

    [obstRow,obstColumn]=np.where(obstacleMap==-900)
    rowLength=obstRow.shape
    for i in range(0,rowLength[0]):
        closeGoal[obstRow[i],obstColumn[i]]=-900

    run=1
    minValue=0
    while run:
        [obstRow,obstColumn]=np.where(closeGoal==minValue)
        rowLength=obstRow.shape

        if rowLength[0]>0:
            for i in range(0,rowLength[0]):
                cost=closeGoal[obstRow[i],obstColumn[i]]
                for e in range (-1,2):
                    for f in range (-1,2):
                        if  (obstRow[i]+e)>=0 and (obstRow[i]+e)<size[0] and (obstColumn[i]+f)>=0 and (obstColumn[i]+f)<size[1] and closeGoal[obstRow[i]+e,obstColumn[i]+f]==-1:
                            if e==0 or f==0:
                                closeGoal[obstRow[i]+e,obstColumn[i]+f]=cost+1
                            else:
                                closeGoal[obstRow[i]+e,obstColumn[i]+f]=cost+2**(0.5)

            if closeGoal[np.where(closeGoal > minValue)].shape[0] > 0:#(closeGoal==-1).sum>0:
                #A=[~np.isnan(closeGoal)]
                minValue=np.nanmin(closeGoal[np.where(closeGoal > minValue)])
                #print minValue
            else:
                run=0

        else:
            run=0
    closeGoal[np.where(closeGoal==-900)]=float('NaN')
    closeGoal=closeGoal+1
    return closeGoal


def coverageMap(astarMap, coveringMap, alpha, startNode, goalNode, map_msg):
    #return obstacleMap
    #print "coveringMap:"
    #print coveringMap

    print "startNode:", startNode
    print "goalNode:", goalNode

    map_height, map_width = astarMap.shape
    img = np.zeros((map_height, map_width, 3))
    for row in range(map_height):
        for col in range(map_width):
            point = astarMap[row][col]
            if point == -1:
                img[row][col] = [100, 100, 100]
            elif point == 0:
                img[row][col] = [255, 255, 255]
            elif point == 100:
                img[row][col] = [0, 0, 0]
            elif point == 70:
                img[row][col] = [30, 30, 30]
            elif point == -2:
                img[row][col] = [0, 255, 0]
    cv2.imwrite("coverageMap_astarMap.png", img)

    map_height, map_width = coveringMap.shape
    img = np.zeros((map_height, map_width, 3))
    for row in range(map_height):
        for col in range(map_width):
            point = coveringMap[row][col]
            if point == -1:
                img[row][col] = [100, 100, 100]
            elif point == 0:
                img[row][col] = [255, 255, 255]
            elif point == 100:
                img[row][col] = [0, 0, 0]
            elif point == 70:
                img[row][col] = [30, 30, 30]
            elif point == -2:
                img[row][col] = [0, 255, 0]
    img[goalNode[0], goalNode[1]] = [255, 0, 0]
    cv2.imwrite("coverageMap1.png", img)






    # set all nodes outside of the 8x8 square to 100 (= obstacle):
    x_min = -4
    x_max = 4
    y_min = -4
    y_max = 4
    #
    temp = pos_2_map_index_mattias(map_msg, [x_min, y_min])
    x_min_ind = temp[0]
    y_min_ind = temp[1]
    temp = pos_2_map_index_mattias(map_msg, [x_max, y_max])
    x_max_ind = temp[0]
    y_max_ind = temp[1]
    #
    if x_max_ind < map_width:
        coveringMap[:, x_max_ind:] = 100
    if x_min_ind > 0:
        coveringMap[:, 0:x_min_ind] = 100
    if y_max_ind < map_height:
        coveringMap[y_max_ind:, :] = 100
    if y_min_ind > 0:
        coveringMap[0:y_min_ind, :] = 100

    map_height, map_width = coveringMap.shape
    img = np.zeros((map_height, map_width, 3))
    for row in range(map_height):
        for col in range(map_width):
            point = coveringMap[row][col]
            if point == -1:
                img[row][col] = [100, 100, 100]
            elif point == 0:
                img[row][col] = [255, 255, 255]
            elif point == 100:
                img[row][col] = [0, 0, 0]
            elif point == 70:
                img[row][col] = [30, 30, 30]
            elif point == -2:
                img[row][col] = [0, 255, 0]
    cv2.imwrite("coverageMap2.png", img)







    if np.count_nonzero(np.nonzero(coveringMap==0))==0:
        print "Map already visited"
        return [-200, -200]
    else:
        coveringMap[coveringMap==70]=-900
        coveringMap[coveringMap==100]=-900
        coveringMap[coveringMap==-1]=-900

        map_height, map_width = coveringMap.shape
        img = np.zeros((map_height, map_width, 3))
        for row in range(map_height):
            for col in range(map_width):
                point = coveringMap[row][col]
                if point == 0:
                    img[row][col] = [255, 255, 255]
                elif point == -900:
                    img[row][col] = [0, 0, 0]
                elif point == -2:
                    img[row][col] = [0, 255, 0]
        cv2.imwrite("coverageMap3.png", img)

        closeObst=clObMap(coveringMap)
        closeGoal=clGoMap(coveringMap, goalNode)

        costMap1=np.add( closeGoal, np.multiply(alpha,closeObst))##print for kostnadskarta
        costMap1[coveringMap==-2]=float('NaN')
        pathX=[]
        pathY=[]

        walkingNode=startNode
        s=0
        map_size=coveringMap.shape
        visitedMap = np.full(map_size,1)
        pathMap = np.full(map_size,0)
        while np.count_nonzero(coveringMap == 0) != (np.count_nonzero(visitedMap == 0)+1):
            print "##########################################################"
            print "while loop in coverageMap"
            #print walkingNode
            #print (walkingNode[1]==goalNode[1] and walkingNode[0]==goalNode[0]==0)
            costMap=np.copy(costMap1)
            newRow=[]
            newCol=[]
            visitedMap[walkingNode[0], walkingNode[1]]=0
            s=s+1
            if walkingNode[0]==0:
                a=0
            else:
                a=-1

            if walkingNode[1]==0:
                b=0
            else:
                b=-1


            if walkingNode[0]==map_size[0]:
                c=0
            else:
                c=1

            if walkingNode[1]==map_size[1]:
                d=0
            else:
                d=1

            partMap=costMap[walkingNode[0]+a:walkingNode[0]+c+1,walkingNode[1]+b:walkingNode[1]+d+1]
            partMap[-a,-b]=0
            partVisitedMap=np.multiply(visitedMap[walkingNode[0]+a:walkingNode[0]+c+1,walkingNode[1]+b:walkingNode[1]+d+1],partMap)

            unVisited=np.where(np.multiply(visitedMap,~np.isnan(costMap))!=0)

            if np.nansum(partVisitedMap)!=0:
                biggest=np.nanmax(partVisitedMap[np.where(partVisitedMap!=0)])
                new=np.where(partVisitedMap==biggest)
                newRow=new[0]
                newCol=new[1]

                ##### borde nog kolla om walkingNode_small ligger i hinder, isf ta den narmsta fria punkten

                walkingNode=[walkingNode[0]+newRow[0]+a,walkingNode[1]+newCol[0]+b] #anpassa beroe p storlek a,b,c,d
                print "not astar: walkingNode:", walkingNode
                print "not astar: coveringMap[walkingNode[0], walkingNode[1]]:", coveringMap[walkingNode[0], walkingNode[1]]
                walkingNode_pos = map_index_2_pos_mattias(map_msg, [walkingNode[1], walkingNode[0]])
                print "not astar: walkingNode_pos:", walkingNode_pos
                walkingNode_small = pos_2_map_index_small(map_msg, walkingNode_pos)
                print "not astar: before:  walkingNode_small:", walkingNode_small
                print "not astar: before: walkingNode_pos after conversion:", map_index_2_pos_small(map_msg, walkingNode_small)
                print "not astar: before: astarMap[walkingNode_small[1], walkingNode_small[0]]:", astarMap[walkingNode_small[1], walkingNode_small[0]]


                if astarMap[walkingNode_small[1], walkingNode_small[0]] == 100:
                    print "########################################################"
                    print "GOAL node is not free, get the closest free node!"
                    print "########################################################"
                    temp = np.nonzero(astarMap == 0)
                    x = temp[1]
                    y = temp[0]
                    distances = (x-walkingNode_small[0])**2 + (y-walkingNode_small[1])**2
                    goalNode = np.argmin(distances)
                    walkingNode_small = [x[goalNode], y[goalNode]]

                    print "not astar: after:  walkingNode_small:", walkingNode_small
                    print "not astar: after: walkingNode_pos after conversion:", map_index_2_pos_small(map_msg, walkingNode_small)
                    print "not astar: after: astarMap[walkingNode_small[1], walkingNode_small[0]]:", astarMap[walkingNode_small[1], walkingNode_small[0]]




                pathX.append(walkingNode_small[1]) ######################### BORDE VAL VARA walkingNode_small?!!?!?!
                pathY.append(walkingNode_small[0]) ######################### BORDE VAL VARA walkingNode_small?!!?!?!
                pathMap[walkingNode[0],walkingNode[1]]=s

            elif unVisited[0].shape[0] > 0:
                #print "length:"
                length=(unVisited[0][:]-walkingNode[0])**(2)+(unVisited[1][:]-walkingNode[1])**(2)
                #print length
                minlength=np.nanmin(length)
                index=np.where(minlength==length)
                newNode= [int(unVisited[0][index][0]),int(unVisited[1][index][0])]

                ##### borde nog kolla om walkingNode_small eller newNode_small ligger i hinder, isf ta den narmsta fria punkten

                #In with A* here to fing path to node
                print "newNode:", newNode
                print "coveringMap[newNode[0], newNode[1]]:", coveringMap[newNode[0], newNode[1]]
                newNode_pos = map_index_2_pos_mattias(map_msg, [newNode[1], newNode[0]])
                print "newNode_pos:", newNode_pos
                newNode_small = pos_2_map_index_small(map_msg, newNode_pos)
                print "newNode_small:", newNode_small
                print "newNode_pos after conversion:", map_index_2_pos_small(map_msg, newNode_small)
                walkingNode_pos = map_index_2_pos_mattias(map_msg, [walkingNode[1], walkingNode[0]])
                walkingNode_small = pos_2_map_index_small(map_msg, walkingNode_pos)
                print "astarMap[walkingNode_small[1], walkingNode_small[0]]:", astarMap[walkingNode_small[1], walkingNode_small[0]]
                print "astarMap[newNode_small[1], newNode_small[0]]:", astarMap[newNode_small[1], newNode_small[0]]



                if astarMap[walkingNode_small[1], walkingNode_small[0]] == 100:
                    print "########################################################"
                    print "START node is not free, get the closest free node!"
                    print "########################################################"
                    temp = np.nonzero(astarMap == 0)
                    x = temp[1]
                    y = temp[0]
                    distances = (x-walkingNode_small[0])**2 + (y-walkingNode_small[1])**2
                    goalNode = np.argmin(distances)
                    walkingNode_small = [x[goalNode], y[goalNode]]

                    print "after:  walkingNode_small:", walkingNode_small
                    print "after: walkingNode_pos after conversion:", map_index_2_pos_small(map_msg, walkingNode_small)
                    print "after: astarMap[walkingNode_small[1], walkingNode_small[0]]:", astarMap[walkingNode_small[1], walkingNode_small[0]]


                if astarMap[newNode_small[1], newNode_small[0]] == 100:
                    print "########################################################"
                    print "GOAL node is not free, get the closest free node!"
                    print "########################################################"
                    temp = np.nonzero(astarMap == 0)
                    x = temp[1]
                    y = temp[0]
                    distances = (x-newNode_small[0])**2 + (y-newNode_small[1])**2
                    goalNode = np.argmin(distances)
                    newNode_small = [x[goalNode], y[goalNode]]

                    print "after: newNode_small:", newNode_small
                    print "after: newNode_pos after conversion:", map_index_2_pos_small(map_msg, newNode_small)
                    print "after: astarMap[newNode_small[1], newNode_small[0]]:", astarMap[newNode_small[1], newNode_small[0]]




                starPath = astar_func([newNode_small[1], newNode_small[0]], [walkingNode_small[1], walkingNode_small[0]], np.copy(astarMap))
                if starPath is None:
                    visitedMap[newNode[0], newNode[1]]=0
                else:
                    starPath = starPath[0]
                    #print starPath
                    size = starPath[0].shape
                    #print size
                    for p in range(0, size[0]):
                        pathX.append(starPath[0][p])
                        pathY.append(starPath[1][p])

                    walkingNode=newNode
                    pathMap[walkingNode[0],walkingNode[1]]=s

            else:
                break

            #print "pathX:"
            #print pathX
            #print "pathY:"
            #print pathY

        path = raw_path_2_path_small([np.array(pathX), np.array(pathY)], map_msg) ######################## [X, Y]????????
        print "coverageMap: path:"
        print path
        return path

        #print costMap
        #return pathMap

def find_goal(obstacleMapG, start):
    goalNode = []
    #goalNodeX = start[0]
    #goalNodeY = obstacleMapG.shape[1]-start[1]
    #goalNode = [goalNodeX,goalNodeY]
    #GoalNode as far away as possible

    unVisited = np.where(obstacleMapG == 0)
    length = (unVisited[0]-start[0])**(2)+(unVisited[1]-start[1])**(2)
    maxlength = np.nanmax(length)
    index = np.where(maxlength == length)
    goalNode = [unVisited[0][index][0], unVisited[1][index][0]]

    print goalNode[0]

    print "{{{{{{{{{{{{{{{{{{{{{{{}}}}}}}}}}}}}}}}}}}}}}"
    print "{{{{{{{{{{{{{{{{{{{{{{{}}}}}}}}}}}}}}}}}}}}}}"
    print "{{{{{{{{{{{{{{{{{{{{{{{}}}}}}}}}}}}}}}}}}}}}}"
    print "goalNode for Covering Tiles: %f %f" %(goalNode[0], goalNode[1])
    print "{{{{{{{{{{{{{{{{{{{{{{{}}}}}}}}}}}}}}}}}}}}}}"
    print "{{{{{{{{{{{{{{{{{{{{{{{}}}}}}}}}}}}}}}}}}}}}}"
    print "{{{{{{{{{{{{{{{{{{{{{{{}}}}}}}}}}}}}}}}}}}}}}"

    return goalNode

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
        msg.data = [-0.2, 0, 0.2, 0]
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
        OBSTACLE_EXPAND_SIZE = 10
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
        x_min = -4
        x_max = 4
        y_min = -4
        y_max = 4
        #
        map_msg = msg_obj
        temp = pos_2_map_index(map_msg, [x_min, y_min])
        x_min_ind = temp[0]
        y_min_ind = temp[1]
        temp = pos_2_map_index(map_msg, [x_max, y_max])
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
        MIN_NR_OF_OBSTACLES = 10
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






        whites = np.count_nonzero(np.nonzero(map_matrix_expand == 0))
        grays = np.count_nonzero(np.nonzero(map_matrix_expand == -1))
        percentage = float(whites)/float(whites + grays)
        print percentage
        if percentage > 0.98:
            print "HALLA ELLLER!"






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
            pos_index = pos_2_map_index(map_msg, pos)
            pos_index_small = pos_2_map_index_small(map_msg, pos)

            print "pos:", pos
            print "pos index:", pos_index


            if self.mode==1:
                goal_pos_index = frontier_func(np.copy(map_matrix), pos_index, map_msg)
                if goal_pos_index==[-1000, -1000]:
                    self.mode=2
                    self.get_path()
                else:
                    self.goal_pos_index = goal_pos_index
                    goal_pos = map_index_2_pos(map_msg, goal_pos_index)
                    print "goal index in FRONTIER map:", goal_pos_index
                    print "goal pos:", goal_pos
                    print "value of goal node in FRONTIER map:", map_matrix[goal_pos_index[1], goal_pos_index[0]]

                    goal_pos_index_small = pos_2_map_index_small(map_msg, goal_pos)
                    print "goal index in ASTAR map:", goal_pos_index_small
                    print "goal pos corresponding to goal index in ASTAR map:", map_index_2_pos_small(map_msg, goal_pos_index_small)
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
                        print "goal pos corresponding to goal index in ASTAR map:", map_index_2_pos_small(map_msg, goal_pos_index_small)
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
                        print "pos corresponding to start index in ASTAR map:", map_index_2_pos_small(map_msg, pos_index_small)
                        print "value of start node in ASTAR map:", map_matrix_small[pos_index_small[1], pos_index_small[0]]

                    raw_path = astar_func([goal_pos_index_small[1], goal_pos_index_small[0]],
                                [pos_index_small[1], pos_index_small[0]], np.copy(map_matrix_small))

                    if raw_path is not None:
                        self.path = raw_path[1]

                        print "raw_path[0]:"
                        print raw_path[0]
                        print "raw_path[1]:"
                        print raw_path[1]

                        #path = raw_path_2_path(raw_path[0], map_msg)
                        path = raw_path_2_path_small(raw_path[0], map_msg)
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
                        pos_index_small = pos_2_map_index_small(map_msg, pos)
                        temp = np.nonzero(map_matrix_small == 0)
                        x = temp[1]
                        y = temp[0]
                        distances = (x-pos_index_small[0])**2 + (y-pos_index_small[1])**2
                        goalNode = np.argmin(distances[np.where(distances > 1)])
                        goal_pos_index_small = [x[goalNode], y[goalNode]]

                        goal_pos = map_index_2_pos_small(map_msg, goal_pos_index_small)
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

                pos_index_mattias = pos_2_map_index_mattias(map_msg, pos)

                goalNodeCov = find_goal(obstacleMap2, [pos_index_mattias[1], pos_index_mattias[0]])
                #SCALING av startnider till ''
                #goalNodeCov = pos_2_map_index_mattias(map_msg, [0,0])
                coverPath = coverageMap(np.copy(map_matrix_small), obstacleMap2, alpha, [pos_index_mattias[1], pos_index_mattias[0]], goalNodeCov, map_msg)
                print "COVERING MODE finishED"
                #print coverPath
                return coverPath

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
