import numpy as np
import cv2
from operator import xor

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
                elif (prevX==(walkingNode[0]+1)) and (prevY==(walkingNode[1]+1)) or (prevX==(walkingNode[0]-1)) and (prevY==(walkingNode[1]-1)):
                    if dir!=2:
                        xRoute.append((walkingNode[0]))
                        yRoute.append(walkingNode[1])
                        dir=2
                else:
                    if dir!=-2:
                        xRoute.append((walkingNode[0]))
                        yRoute.append(walkingNode[1])
                        dir=-2

                walkingNode=[prevX, prevY]
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
