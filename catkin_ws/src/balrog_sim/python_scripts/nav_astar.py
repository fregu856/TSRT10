# NOTE! This is just a copy of balrog/python_scripts/nav_astar.py

''' astar finds shortest path from a->b in a area with obstacles.
    Input is goal and start node written as [Int indexOfTheRow, Int IndexOfTheCol] along with the obstacleMap (np matrix containing doubles or floats) with all obstacles including safety distance.
    Output is indices of a filtered path along with the full path  [[[Int rowIndicesFiltered] [Int colIndicesFiltered]][[Int rowIndicesFull] [Int colIndicesFull]]] .
    In this code x corresponds to the row and y to the column.
'''

import numpy as np
import cv2
from operator import xor

def astar_func(goalNode, startNode, obstacleMap):

    print "start of Astar!"

    slamMap = np.copy(obstacleMap)
    rows, cols = slamMap.shape

    # debug:
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
    # img[goalNode[0]][goalNode[1]] = [0, 255, 0]
    # cv2.imwrite("astar.png", img)

    [xmax, ymax] = obstacleMap.shape
    fCostMap = np.full((xmax, ymax), float('nan'))
    fCostMapOld = np.full((xmax, ymax), float('nan'))
    hCostMap = np.zeros((xmax, ymax))
    gCostMap = np.full((xmax, ymax), float('nan'))
    gCostMapOld = np.full((xmax, ymax), float('nan'))
    checkedMap = np.zeros((xmax, ymax)) #checked Nodes (has been current node)
    fValueMap = np.zeros((xmax, ymax)) #nodes having a f-cost
    fromXNode = np.zeros((xmax, ymax)) #which x-node it came from
    fromYNode = np.zeros((xmax, ymax)) #which y-node it came from
    found = 0
    currentNode = startNode

    #calculate the map with h-costs, (distance to goal node)
    hDiffXMap = np.zeros(obstacleMap.shape)
    hDiffYMap = np.zeros(obstacleMap.shape)
    for xCounter in range(0,xmax):
        hDiffXMap[xcounter, :] = abs(xCounter - goalNode[0])
    for yCounter in range(0,ymax):
        hDiffYMap[:,ycounter] = abs(yCounter - goalNode[1])

    #euclidean distance to goal node
    hCostMap = 2 ** (0.5) * np.minimum(hDiffXMap, hDiffYMap) + np.maximum(hDiffXMap, hDiffYMap) - np.minimum(hDiffXMap, hDiffYMap)
    gCostMapOld[currentNode[0], currentNode[1]] = 0

    #run until path found or to max iterations
    for noLoops in range(15000):
        checkedMap[currentNode[0], currentNode[1]] = 1    #current node has been checked
        for rowCounter in range(-1,2):
            for colCounter in range(-1,2):
                oldValue = []

                #calc gcost for current index
                b = gCostMapOld[currentNode[0], currentNode[1]]

                #check if inside of size, if straight movement and not an obstacle
                if ((currentNode[0] + rowCounter) > -1 and (currentNode[1] + colCounter) > -1 and
                    (currentNode[0] + rowCounter) < xmax and (currentNode[1] + colCounter) < ymax and xor(colCounter == 0, rowCounter == 0)
                    and obstacleMap[(currentNode[0] + rowCounter),(currentNode[1] + colCounter)] != 100):

                    gCostMap[currentNode[0] + rowCounter, (currentNode[1] + colCounter)] = b + 1 #set the g-cost
                    oldValue = gCostMapOld[currentNode[0] + rowCounter, (currentNode[1] + colCounter)]

                    #check if the old value is smaller, then save that information
                    if (gCostMap[currentNode[0] + rowCounter, (currentNode[1] + colCounter)] > oldValue):
                        gCostMap[currentNode[0] + rowCounter, (currentNode[1] + colCounter)] = oldValue

                    #save which index the cost came from
                    else:
                        fromXNode[currentNode[0] + rowCounter, (currentNode[1] + colCounter)] = currentNode[0]
                        fromYNode[currentNode[0] + rowCounter, (currentNode[1] + colCounter)] = currentNode[1]
                        fValueMap[currentNode[0] + rowCounter, (currentNode[1] + colCounter)] = 1 #current node has a f-value

                #check if inside of size, if diagonal movement and not an obstacle
                elif ((currentNode[0] + rowCounter) > -1 and (currentNode[1] + colCounter) > -1 and
                    (currentNode[0] + rowCounter) < xmax and (currentNode[1] + colCounter) < ymax and
                    rowCounter != 0 and colCounter != 0  and obstacleMap[(currentNode[0] + rowCounter), (currentNode[1] + colCounter)] != 100):

                    gCostMap[currentNode[0] + rowCounter, (currentNode[1] + colCounter)] = b + 2 ** (0.5) #set g-cost
                    oldValue = gCostMapOld[currentNode[0] + rowCounter, (currentNode[1] + colCounter)]

                    #check if the old value is smaller, then save that information
                    if (gCostMap[currentNode[0] + rowCounter, (currentNode[1] + colCounter)] > oldValue):
                        gCostMap[currentNode[0] + rowCounter, (currentNode[1] + colCounter)] = oldValue
                    #save which index the cost came from
                    else:
                        fromXNode[currentNode[0] + rowCounter, (currentNode[1] + colCounter)] = currentNode[0]
                        fromYNode[currentNode[0] + rowCounter, (currentNode[1] + colCounter)] = currentNode[1]
                        fValueMap[currentNode[0] + rowCounter, (currentNode[1] + colCounter)] = 1

        gCostMap[currentNode[0], currentNode[1]] = b #set middle node to b (might not be needed)
        gCostMap[startNode[0], startNode[1]] = 0 #start node should always be 0
        fCostMap = np.add(gCostMap, hCostMap) #calc f-cost

        minPosition = [] #the indices which has min-value [[minRow] [minCol]]
        minRow = []   #the rows which has min-value
        minColumn = []  #the columns which has min-value
        lookH = [] #take out the h-costs
        minHValue = [] #the smallest h-cost found


        numMinPoints = []

        #find the node with the lowest set cost that not has been checked or is an obstacle
        searchFCost = fCostMap * fValueMap * (checkedMap != 1) * (obstacleMap != 100)
        if np.nansum(searchFCost) > 0:
            minPosition=np.where(searchFCost == np.nanmin(searchFCost[np.nonzero(searchFCost)])) #takes out the indices that has the min-value
        else:
            print "np.nansum(searchFCost)>0 is NOT true, break!" #goal not reachable
            break

        numMinPoints = np.shape(minPosition)
        minRow = minPosition[0]
        minColumn = minPosition[1]

        #when many node has the same cost, choose the one with smallest h-cost
        for nodeCounter in range(0, numMinPoints[1]):
            lookH.append(hCostMap[minRow[nodeCounter], minColumn[nodeCounter]])

        minHValue = np.where(lookH == np.min(lookH))

        currentNode = [minRow[minHValue[0][0]], minColumn[minHValue[0][0]]] #change current node to the one with smallest f and h-cost

        #write over the old maps
        gCostMapOld = np.copy(gCostMap)
        fCostMapOld = np.copy(fCostMap)


        if fValueMap[goalNode[0], goalNode[1]] == 1: #goal node has been reached
            found = 1
            walkingNode = goalNode

            #full paths
            wholeXroute = []
            wholeYroute = []

            #filtered paths
            xRoute = []
            yRoute = []
            wholeXroute.append(goalNode[0])
            wholeYroute.append(goalNode[1])

            dir = 0
            #iterate backwards until start node is found
            while walkingNode[0] != startNode[0] or walkingNode[1] != startNode[1]:
                #check in fromX/YNode which index was the previous
                prevX = int(fromXNode[walkingNode[0], walkingNode[1]])
                prevY = int(fromYNode[walkingNode[0], walkingNode[1]])
                wholeXroute.append(prevX)
                wholeYroute.append(prevY)

                #only append if the direction of motion has changed
                if prevX == walkingNode[0]: #horizontal movement
                    if dir != 5:
                        xRoute.append((walkingNode[0]))
                        yRoute.append(walkingNode[1])
                        dir = 5
                elif prevY == walkingNode[1]: #vertical movement
                    if dir != 3:
                        xRoute.append((walkingNode[0]))
                        yRoute.append(walkingNode[1])
                        dir = 3
                elif (prevX == (walkingNode[0] + 1)) and (prevY == (walkingNode[1] + 1))
                    or (prevX == (walkingNode[0] - 1)) and (prevY == (walkingNode[1] - 1)): # upp-left diagonal movement
                    if dir != 2:
                        xRoute.append((walkingNode[0]))
                        yRoute.append(walkingNode[1])
                        dir = 2
                else: # upp-right diagonal movement
                    if dir != -2:
                        xRoute.append((walkingNode[0]))
                        yRoute.append(walkingNode[1])
                        dir = -2

                walkingNode = [prevX, prevY] #write over walking node

            route = [xRoute, yRoute] ############################################################################################################################### before: route=[xRoute, xRoute]

            #flip routes to get from start to goal
            routeS2G = np.fliplr(route)
            wholeRoute = np.fliplr([wholeXroute, wholeYroute])

            # debug:
            # # save obstacleMap as an image, with each point on the A* path
            # # marked in red:
            # map_height, map_width = obstacleMap.shape
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
            # cols = yRoute
            # rows = xRoute
            # for (row, col) in zip(rows, cols):
            #     img[row, col] = [0, 0, 255]
            # img[goalNode[0]][goalNode[1]] = [0, 255, 0]
            # cv2.imwrite("astar_route.png", img)

            print "Astar found a route!"
            print "end of Astar!"
            return routeS2G, wholeRoute

    print "Astar didn't find a route in time!"
    print "end of Astar!"
    return None
