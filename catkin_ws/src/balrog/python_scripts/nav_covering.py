'''All code neccesary for covering mode (mode 2), call find_goal(input) to find
   the goal position and thereafter coveringMap(input) for the whole path '''

import numpy as np
import cv2

from utilities import map_index_2_pos, pos_2_map_index, raw_path_2_path
from nav_astar import astar_func

# size of considered area:
X_MAX = 4 # (NOTE! if this value is modified one also needs to update it in main.py)
X_MIN = -0.2 # (NOTE! if this value is modified one also needs to update it in main.py)
Y_MAX = 4 # (NOTE! if this value is modified one also needs to update it in main.py)
Y_MIN = -0.2 # (NOTE! if this value is modified one also needs to update it in main.py)

# map resolutions:
MAP_RES_ASTAR = 0.25 # (NOTE! if this value is modified one also needs to update it in main.py)
MAP_RES_COVERING = 0.30 # (NOTE! if this value is modified one also needs to update it in main.py)

def clObMap(obstacleMap):
    #Making map, showing closeness to obstacles (used in covering_tiles), takes in obstacle map
    size=obstacleMap.shape
    closeObst=np.full(size,-1.0)
    closeObst[(obstacleMap==-900)]=0 #set obstacles to 0
    #closeObst[obstacleMap==-2]=0 #possibility to also see visited nodes as obstacles

    run=1
    cost=0

    while run:
    #find all nodes with the cost, and set the adjacent to cost+1
        pos=np.where(closeObst==cost)
        rowPos=pos[0]
        colPos=pos[1]
        rowLength=rowPos.shape

        cost=cost+1
        if rowLength[0]>0:
            for nNodes in range(0,rowLength[0]):
                for rowCount in range (-1,2):
                    for colCount in range (-1,2):
                        if  ((rowPos[nNodes]+rowCount)>=0 and (rowPos[nNodes]+rowCount)<size[0] and (colPos[nNodes]+colCount)>=0 and
                            (colPos[nNodes]+colCount)<size[1] and closeObst[rowPos[nNodes]+rowCount,colPos[nNodes]+colCount]==-1): #check if inside of map size
                            closeObst[rowPos[nNodes]+rowCount,colPos[nNodes]+colCount]=cost
        else:
        #stop if all of the map has a cost
            run=0
    #make the cost go from high to low
    return np.nanmax(closeObst)-closeObst

def clGoMap(obstacleMap, goalNode):
    #Making map, showing closeness to goal node (used in covering_tiles), using goalnode as [row,col] and obstacleMap
    size=obstacleMap.shape
    closeGoal=np.full(size,-1.0)
    closeGoal[goalNode[0],goalNode[1]]=0 #cost based on goal node

    #set the nodes where there is obstacle to -900  (closeGoal[(obstacleMap==-900)]=-900 should also work)
    [obstRow,obstColumn]=np.where(obstacleMap==-900)
    rowLength=obstRow.shape
    for i in range(0,rowLength[0]):
        closeGoal[obstRow[i],obstColumn[i]]=-900


    run=1
    minValue=0
    while run:
    #find nodes with minValue and set the adjacent to 1 and sqrt(2)
        [obstRow,obstColumn]=np.where(closeGoal==minValue)
        rowLength=obstRow.shape

        if rowLength[0]>0:
            for  noNodes in range(0,rowLength[0]):
                cost=closeGoal[obstRow[noNodes],obstColumn[noNodes]]
                for rowCount in range (-1,2):
                    for colCount in range (-1,2):
                        #Only set unset nodes within the size
                        if  ((obstRow[noNodes]+rowCount)>=0 and (obstRow[noNodes]+rowCount)<size[0] and
                            (obstColumn[noNodes]+colCount)>=0 and (obstColumn[noNodes]+colCount)<size[1] and
                            closeGoal[obstRow[noNodes]+rowCount,obstColumn[noNodes]+colCount]==-1):
                            #can set cost depending on straight or diagonal movement
                            if rowCount==0 or colCount==0:
                                closeGoal[obstRow[noNodes]+rowCount,obstColumn[noNodes]+colCount]=cost+1
                            else:
                                closeGoal[obstRow[noNodes]+rowCount,obstColumn[noNodes]+colCount]=cost+2**(0.5)

            if closeGoal[np.where(closeGoal > minValue)].shape[0] > 0:#find the new min value bigger than the current one
                minValue=np.nanmin(closeGoal[np.where(closeGoal > minValue)])
                #print minValue
            else:
                run=0 #whole map has been set

        else:
            run=0
    #set the obstacles to nan instead, (gives error if used at first, more changes is needed itc)
    closeGoal[np.where(closeGoal==-900)]=float('NaN')
    closeGoal=closeGoal+1
    return closeGoal

def coverageMap(astarMap, coveringMap, alpha, startNode, goalNode, map_origin):
    '''sends out a path for covering a whole (a big or many small) surfaces.
    takes in the map for using a* (np matrix), another map vhere the visited tiles are set to -2 (np matrix), the tuning parameter alpha,
    the start and goal node according to coveringMap (input as [Int indexRow, Int indexCol]), and the parameters making it possible to change index from the different maps
    Output is indices of a filtered path for the a* map along with the full path  [[[Int rowIndicesFiltered] [Int colIndicesFiltered]][[Int rowIndicesFull] [Int colIndicesFull]]] .
    '''
    #return obstacleMap
    #print "coveringMap:"
    #print coveringMap

    print "startNode:", startNode
    print "goalNode:", goalNode

    map_height, map_width = coveringMap.shape

    # debug:
    # map_height, map_width = coveringMap.shape
    # img = np.zeros((map_height, map_width, 3))
    # for row in range(map_height):
    #     for col in range(map_width):
    #         point = coveringMap[row][col]
    #         if point == -1:
    #             img[row][col] = [100, 100, 100]
    #         elif point == 0:
    #             img[row][col] = [255, 255, 255]
    #         elif point == 100:
    #             img[row][col] = [0, 0, 0]
    #         elif point == 70:
    #             img[row][col] = [30, 30, 30]
    #         elif point == -2:
    #             img[row][col] = [0, 255, 0]
    # img[goalNode[0], goalNode[1]] = [255, 0, 0]
    # cv2.imwrite("coverageMap1.png", img)

    # set all nodes outside of the considered area to 100 (= obstacle):
    xMin = X_MIN
    xMax = X_MAX
    yMin = Y_MIN
    yMax = Y_MAX
    #
    temp = pos_2_map_index(map_origin, MAP_RES_COVERING, [xMin, yMin])
    xMinInd = temp[0]
    yMinInd = temp[1]
    temp = pos_2_map_index(map_origin, MAP_RES_COVERING, [xMax, yMax])
    xMaxInd = temp[0]
    yMaxInd = temp[1]
    #
    if xMaxInd < map_width:
        coveringMap[:, xMaxInd:] = 100
    if xMinInd > 0:
        coveringMap[:, 0:xMinInd] = 100
    if yMaxInd < map_height:
        coveringMap[yMaxInd:, :] = 100
    if yMinInd > 0:
        coveringMap[0:yMinInd, :] = 100

    # debug:
    # map_height, map_width = coveringMap.shape
    # img = np.zeros((map_height, map_width, 3))
    # for row in range(map_height):
    #     for col in range(map_width):
    #         point = coveringMap[row][col]
    #         if point == -1:
    #             img[row][col] = [100, 100, 100]
    #         elif point == 0:
    #             img[row][col] = [255, 255, 255]
    #         elif point == 100:
    #             img[row][col] = [0, 0, 0]
    #         elif point == 70:
    #             img[row][col] = [30, 30, 30]
    #         elif point == -2:
    #             img[row][col] = [0, 255, 0]
    # cv2.imwrite("coverageMap2.png", img)

    if np.count_nonzero(np.nonzero(coveringMap==0))<5:
        print "Map already visited"
        return None
    else:
        #due to different numbers in different maps see all as following as obstacles
        coveringMap[coveringMap==70]=-900 #safety distance from obstacles
        coveringMap[coveringMap==100]=-900 #real obstacles
        coveringMap[coveringMap==-1]=-900 #nodes outside of map

        # debug:
        # map_height, map_width = coveringMap.shape
        # img = np.zeros((map_height, map_width, 3))
        # for row in range(map_height):
        #     for col in range(map_width):
        #         point = coveringMap[row][col]
        #         if point == 0:
        #             img[row][col] = [255, 255, 255]
        #         elif point == -900:
        #             img[row][col] = [0, 0, 0]
        #         elif point == -2:
        #             img[row][col] = [0, 255, 0]
        # cv2.imwrite("coverageMap3.png", img)

        closeObst=clObMap(np.copy(coveringMap))
        closeGoal=clGoMap(coveringMap, goalNode)

        # costMap=closeGoal+alpha*closeObstacle
        costMap1=np.add( closeGoal, np.multiply(alpha,closeObst))
        costMap1[coveringMap==-2]=float('NaN')
        pathX=[]
        pathY=[]

        walkingNode=startNode
        noLoops=0
        map_size=coveringMap.shape
        visitedMap = np.full(map_size, 1.0) #where it has been, in covering mode, 0 if true
        pathMap = np.full(map_size, 0.0) #pathMap shows in which order it goes to the nodes, good for debugging
        while True:
            print "##########################################################"
            print "while loop in coverageMap"
            #print walkingNode
            #print (walkingNode[1]==goalNode[1] and walkingNode[0]==goalNode[0]==0)
            costMap=np.copy(costMap1)
            newRow=[]
            newCol=[]

            noLoops=noLoops+1

            #checks if index is too small or big, adjust search maxcost according to that
            if walkingNode[0]==0:
                top=0
            elif walkingNode[0]==1:
                top=-1
            else:
                top=-2

            if walkingNode[1]==0:
                left=0
            elif walkingNode[1]==1:
                left=-1
            else:
                left=-2

            if walkingNode[0]==map_size[0]:
                bottom=0
            elif walkingNode[0]==map_size[0]-1:
                bottom=1
            else:
                bottom=2

            if walkingNode[1]==map_size[1]:
                right=0
            elif walkingNode[1]==map_size[0]-1:
                right=1
            else:
                right=2

            # see the eight adjacent nodes as visited
            visitedMap[walkingNode[0]+top+1:walkingNode[0]+bottom,walkingNode[1]+left+1:walkingNode[1]+right]=0
            #take out a part of visited map 5x5 nodes with walking node in the middle
            partMap=costMap[walkingNode[0]+top:walkingNode[0]+bottom+1,walkingNode[1]+left:walkingNode[1]+right+1]
            #don't search with the current node (set to 0)
            partMap[-top,-left]=0
            #multiply the two maps, the result is the cost for all unvisited nodes
            partVisitedMap=(np.multiply(visitedMap[walkingNode[0]+top:walkingNode[0]+bottom+1,
                walkingNode[1]+left:walkingNode[1]+right+1],partMap))

            #look which nodes that are left to be visited
            unVisited=np.where(np.multiply(visitedMap,~np.isnan(costMap))!=0)

            if np.nansum(partVisitedMap)!=0:
                #if possible to go to a node in the 5x5 area, find the one with highest cost
                biggest=np.nanmax(partVisitedMap[np.where(partVisitedMap!=0)])
                new=np.where(partVisitedMap==biggest)
                newRow=new[0]
                newCol=new[1]

                walkingNode=[walkingNode[0]+newRow[0]+top,walkingNode[1]+newCol[0]+left] #anpassa beroe p storlek top,left,bottom,right
                # print "not astar: walkingNode:", walkingNode
                # print "not astar: coveringMap[walkingNode[0], walkingNode[1]]:", coveringMap[walkingNode[0], walkingNode[1]]
                walkingNode_pos = map_index_2_pos(map_origin, MAP_RES_COVERING, [walkingNode[1], walkingNode[0]])
                # print "not astar: walkingNode_pos:", walkingNode_pos
                walkingNode_small = pos_2_map_index(map_origin, MAP_RES_ASTAR, walkingNode_pos)
                # print "not astar: before:  walkingNode_small:", walkingNode_small
                # print "not astar: before: walkingNode_pos after conversion:", map_index_2_pos(map_origin, MAP_RES_ASTAR, walkingNode_small)
                # print "not astar: before: astarMap[walkingNode_small[1], walkingNode_small[0]]:", astarMap[walkingNode_small[1], walkingNode_small[0]]

                if astarMap[walkingNode_small[1], walkingNode_small[0]] == 100:
                    # print "########################################################"
                    # print "GOAL node is not free, get the closest free node!"
                    # print "########################################################"
                    temp = np.nonzero(astarMap == 0)
                    x = temp[1]
                    y = temp[0]
                    distances = (x-walkingNode_small[0])**2 + (y-walkingNode_small[1])**2
                    goalNode = np.argmin(distances)
                    walkingNode_small = [x[goalNode], y[goalNode]]

                    # print "not astar: after:  walkingNode_small:", walkingNode_small
                    # print "not astar: after: walkingNode_pos after conversion:", map_index_2_pos(map_origin, MAP_RES_ASTAR, walkingNode_small)
                    # print "not astar: after: astarMap[walkingNode_small[1], walkingNode_small[0]]:", astarMap[walkingNode_small[1], walkingNode_small[0]]

                pathX.append(walkingNode_small[1])
                pathY.append(walkingNode_small[0])
                pathMap[walkingNode[0],walkingNode[1]]=noLoops

            elif unVisited[0].shape[0] > 0:
            #else when there is other free nodes in the map left go to the closest node and/or the one with the highest cost (change the constants for length)

                length=(unVisited[0][:]-walkingNode[0])**(2)+(unVisited[1][:]-walkingNode[1])**(2) #+ 0.2*costMap[unVisited[0][:],unVisited[1][:]]
                minlength=np.nanmin(length)
                index=np.where(minlength==length)
                newNode= [int(unVisited[0][index][0]),int(unVisited[1][index][0])]

                ##### borde nog kolla om walkingNode_small eller newNode_small ligger i hinder, isf ta den narmsta fria punkten

                #In with A* here to fing path to node
                # print "newNode:", newNode
                # print "coveringMap[newNode[0], newNode[1]]:", coveringMap[newNode[0], newNode[1]]
                newNode_pos = map_index_2_pos(map_origin, MAP_RES_COVERING, [newNode[1], newNode[0]])
                # print "newNode_pos:", newNode_pos
                newNode_small = pos_2_map_index(map_origin, MAP_RES_ASTAR, newNode_pos)
                # print "newNode_small:", newNode_small
                # print "newNode_pos after conversion:", map_index_2_pos(map_origin, MAP_RES_ASTAR, newNode_small)
                #change index
                walkingNode_pos = map_index_2_pos(map_origin, MAP_RES_COVERING, [walkingNode[1], walkingNode[0]])
                walkingNode_small = pos_2_map_index(map_origin, MAP_RES_ASTAR, walkingNode_pos)
                # print "astarMap[walkingNode_small[1], walkingNode_small[0]]:", astarMap[walkingNode_small[1], walkingNode_small[0]]
                # print "astarMap[newNode_small[1], newNode_small[0]]:", astarMap[newNode_small[1], newNode_small[0]]

                if astarMap[walkingNode_small[1], walkingNode_small[0]] == 100:
                    # print "########################################################"
                    # print "START node is not free, get the closest free node!"
                    # print "########################################################"
                    temp = np.nonzero(astarMap == 0)
                    x = temp[1]
                    y = temp[0]
                    distances = (x-walkingNode_small[0])**2 + (y-walkingNode_small[1])**2
                    goalNode = np.argmin(distances)
                    walkingNode_small = [x[goalNode], y[goalNode]]

                    # print "after:  walkingNode_small:", walkingNode_small
                    # print "after: walkingNode_pos after conversion:", map_index_2_pos(map_origin, MAP_RES_ASTAR, walkingNode_small)
                    # print "after: astarMap[walkingNode_small[1], walkingNode_small[0]]:", astarMap[walkingNode_small[1], walkingNode_small[0]]

                if astarMap[newNode_small[1], newNode_small[0]] == 100:
                    # print "########################################################"
                    # print "GOAL node is not free, get the closest free node!"
                    # print "########################################################"
                    temp = np.nonzero(astarMap == 0)
                    x = temp[1]
                    y = temp[0]
                    distances = (x-newNode_small[0])**2 + (y-newNode_small[1])**2
                    goalNode = np.argmin(distances)
                    newNode_small = [x[goalNode], y[goalNode]]

                    # print "after: newNode_small:", newNode_small
                    # print "after: newNode_pos after conversion:", map_index_2_pos(map_origin, MAP_RES_ASTAR, newNode_small)
                    # print "after: astarMap[newNode_small[1], newNode_small[0]]:", astarMap[newNode_small[1], newNode_small[0]]

                #add the a* nodes to the path if possible
                starPath = astar_func([newNode_small[1], newNode_small[0]], [walkingNode_small[1], walkingNode_small[0]], np.copy(astarMap))
                if starPath is None:
                    visitedMap[newNode[0], newNode[1]]=0 #cant reach the node
                else:
                    starPath = starPath[0]
                    #print starPath
                    size = starPath[0].shape
                    #print size
                    for p in range(0, size[0]):
                        pathX.append(starPath[0][p])
                        pathY.append(starPath[1][p])

                    walkingNode=newNode
                    pathMap[walkingNode[0],walkingNode[1]]=noLoops

            else:
                break # (break the while True loop)

            #print "pathX:"
            #print pathX
            #print "pathY:"
            #print pathY

        if pathX == []:
            return None

        raw_path = [np.array(pathX), np.array(pathY)]

        pathX=np.fliplr([pathX])[0]
        pathY=np.fliplr([pathY])[0]
        xRoute=[]
        yRoute=[]
        #filtering of path, to increase speed
        #compare two following x and y nodes, to see in which direction it's moving
        dir=0
        for pathCount in range(0,len(pathX)-1):

            prevX=pathX[pathCount+1]
            prevY=pathY[pathCount+1]

            if prevX==pathX[pathCount]: #horizontal movement
                if pathY[pathCount]>prevY:
                    if dir!=5:
                        #if comming from another direction, append
                        xRoute.append(pathX[pathCount])
                        yRoute.append(pathY[pathCount])
                        dir=5
                else:
                    if dir!=4:
                        xRoute.append(pathX[pathCount])
                        yRoute.append(pathY[pathCount])
                        dir=4
            elif prevY==pathY[pathCount]: #vertical movement
                if pathX[pathCount]>prevX:
                    if dir!=3:
                        xRoute.append(pathX[pathCount])
                        yRoute.append(pathY[pathCount])
                        dir=3
                else:
                    if dir!=30:
                        xRoute.append(pathX[pathCount])
                        yRoute.append(pathY[pathCount])
                        dir=30
            #diagonal (uppwards-left)
            elif ((prevX==(pathX[pathCount]+1) and prevY==(pathY[pathCount]+1) ) or (prevX==(pathX[pathCount]+2) and prevY==(pathY[pathCount]+2))
                or (prevX==(pathX[pathCount]-1) and prevY==(pathY[pathCount]-1)) or (prevX==(pathX[pathCount]-2) and prevY==(pathY[pathCount]-2))):
                if (prevX==(pathX[pathCount]+1) and prevY==(pathY[pathCount]+1) ) or (prevX==(pathX[pathCount]+2) and prevY==(pathY[pathCount]+2)):
                    if dir!=2:
                        xRoute.append(pathX[pathCount])
                        yRoute.append(pathY[pathCount])
                        dir=2
                else:
                    if dir!=22:
                        xRoute.append(pathX[pathCount])
                        yRoute.append(pathY[pathCount])
                        dir=22
            #diagonal (uppwards-right)
            elif ((prevX==(pathX[pathCount]+1) and prevY==(pathY[pathCount]-1) ) or (prevX==(pathX[pathCount]+2) and prevY==(pathY[pathCount]-2))
                 or (prevX==(pathX[pathCount]-1) and prevY==(pathY[pathCount]+1)) or (prevX==(pathX[pathCount]-2) and prevY==(pathY[pathCount]+2))):
                if (prevX==(pathX[pathCount]+1) and prevY==(pathY[pathCount]-1) ) or (prevX==(pathX[pathCount]+2) and prevY==(pathY[pathCount]-2)):
                    if dir!=-2:
                        xRoute.append(pathX[pathCount])
                        yRoute.append(pathY[pathCount])
                        dir=-2
                else:
                    if dir!=-19:
                        xRoute.append(pathX[pathCount])
                        yRoute.append(pathY[pathCount])
                        dir=-19
            #all other cases, jump in path or other directions
            else:

                xRoute.append(pathX[pathCount])
                yRoute.append(pathY[pathCount])
                dir=-44

        if xRoute == []:
            return None

        route=[xRoute, yRoute]
        routeS2G=np.fliplr(route)

        return routeS2G, raw_path #filtered path along with full path

def find_goal(obstacleMapG, start):
    #find the node most far away from the startnode (of the reachable)
    unVisited = np.where(obstacleMapG == 0)
    if unVisited[0].shape[0] > 0:
        length = (unVisited[0]-start[0])**(2)+(unVisited[1]-start[1])**(2)
        maxlength = np.nanmax(length)
        index = np.where(maxlength == length)
        goalNode = [unVisited[0][index][0], unVisited[1][index][0]]

        # print "{{{{{{{{{{{{{{{{{{{{{{{}}}}}}}}}}}}}}}}}}}}}}"
        # print "{{{{{{{{{{{{{{{{{{{{{{{}}}}}}}}}}}}}}}}}}}}}}"
        # print "{{{{{{{{{{{{{{{{{{{{{{{}}}}}}}}}}}}}}}}}}}}}}"
        # print "goalNode for Covering Tiles: %f %f" %(goalNode[0], goalNode[1])
        # print "{{{{{{{{{{{{{{{{{{{{{{{}}}}}}}}}}}}}}}}}}}}}}"
        # print "{{{{{{{{{{{{{{{{{{{{{{{}}}}}}}}}}}}}}}}}}}}}}"
        # print "{{{{{{{{{{{{{{{{{{{{{{{}}}}}}}}}}}}}}}}}}}}}}"

        return goalNode
    else:
        return None
