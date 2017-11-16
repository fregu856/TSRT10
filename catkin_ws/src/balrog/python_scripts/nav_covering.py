import numpy as np
import cv2

from utilities import map_index_2_pos, pos_2_map_index, raw_path_2_path
from nav_astar import astar_func

X_MAX = 4 # (NOTE! if this value is modified one also needs to update it in nav_mapping.py)
X_MIN = -4 # (NOTE! if this value is modified one also needs to update it in nav_mapping.py)
Y_MAX = 4 # (NOTE! if this value is modified one also needs to update it in nav_mapping.py)
Y_MIN = -4 # (NOTE! if this value is modified one also needs to update it in nav_mapping.py)

MAP_RES_ASTAR = 0.25 # (NOTE! if this value is modified one also needs to update it in nav_mapping.py)
MAP_RES_COVERING = 0.30 # (NOTE! if this value is modified one also needs to update it in nav_mapping.py)

def clObMap(obstacleMap):
#Making map, showing closeness to obstacles
    size=obstacleMap.shape
    closeObst=np.full(size,-1 )
    closeObst[(obstacleMap==-900)]=0
    closeObst[obstacleMap==-2]=0 #set obstacles to 0

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
    x_min = X_MIN
    x_max = X_MAX
    y_min = Y_MIN
    y_max = Y_MAX
    #
    temp = pos_2_map_index(map_msg, MAP_RES_COVERING, [x_min, y_min])
    x_min_ind = temp[0]
    y_min_ind = temp[1]
    temp = pos_2_map_index(map_msg, MAP_RES_COVERING, [x_max, y_max])
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

        closeObst=clObMap(np.copy(coveringMap))
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
        while True:
            print "##########################################################"
            print "while loop in coverageMap"
            #print walkingNode
            #print (walkingNode[1]==goalNode[1] and walkingNode[0]==goalNode[0]==0)
            costMap=np.copy(costMap1)
            newRow=[]
            newCol=[]
            extraNodes=2


            s=s+1
            if walkingNode[0]==0:
                a=0
            elif walkingNode[0]==1:
                a=-1
            else:
                a=-2



            if walkingNode[1]==0:
                b=0
            elif walkingNode[0]==1:
                b=-1
            else:
                b=-2

            if walkingNode[0]==map_size[0]:
                c=0
            elif walkingNode[0]==map_size[0]-1:
                c=1
            else:
                c=2

            if walkingNode[1]==map_size[1]:
                d=0
            elif walkingNode[0]==map_size[0]-1:
                d=1
            else:
                d=2

            visitedMap[walkingNode[0]+a+1:walkingNode[0]+c,walkingNode[1]+b+1:walkingNode[1]+d]=0
            partMap=costMap[walkingNode[0]+a:walkingNode[0]+c+1,walkingNode[1]+b:walkingNode[1]+d+1]
            #print visitedMap[walkingNode[0]+a+1:walkingNode[0]+c,walkingNode[1]+b+1:walkingNode[1]+d]
            partMap[-a,-b]=0
            partVisitedMap=np.multiply(visitedMap[walkingNode[0]+a:walkingNode[0]+c+1,walkingNode[1]+b:walkingNode[1]+d+1],partMap)
            #print partVisitedMap

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
                walkingNode_pos = map_index_2_pos(map_msg, MAP_RES_COVERING, [walkingNode[1], walkingNode[0]])
                print "not astar: walkingNode_pos:", walkingNode_pos
                walkingNode_small = pos_2_map_index(map_msg, MAP_RES_ASTAR, walkingNode_pos)
                print "not astar: before:  walkingNode_small:", walkingNode_small
                print "not astar: before: walkingNode_pos after conversion:", map_index_2_pos(map_msg, MAP_RES_ASTAR, walkingNode_small)
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
                    print "not astar: after: walkingNode_pos after conversion:", map_index_2_pos(map_msg, MAP_RES_ASTAR, walkingNode_small)
                    print "not astar: after: astarMap[walkingNode_small[1], walkingNode_small[0]]:", astarMap[walkingNode_small[1], walkingNode_small[0]]




                pathX.append(walkingNode_small[1]) ######################### BORDE VAL VARA walkingNode_small?!!?!?!
                pathY.append(walkingNode_small[0]) ######################### BORDE VAL VARA walkingNode_small?!!?!?!
                pathMap[walkingNode[0],walkingNode[1]]=s

            elif unVisited[0].shape[0] > 0:
                #print "length:"
                length=0.8*(unVisited[0][:]-walkingNode[0])**(2)+(unVisited[1][:]-walkingNode[1])**(2) + 0.2*costMap[unVisited[0][:],unVisited[1][:]]
                #print length
                minlength=np.nanmin(length)
                index=np.where(minlength==length)
                newNode= [int(unVisited[0][index][0]),int(unVisited[1][index][0])]

                ##### borde nog kolla om walkingNode_small eller newNode_small ligger i hinder, isf ta den narmsta fria punkten

                #In with A* here to fing path to node
                print "newNode:", newNode
                print "coveringMap[newNode[0], newNode[1]]:", coveringMap[newNode[0], newNode[1]]
                newNode_pos = map_index_2_pos(map_msg, MAP_RES_COVERING, [newNode[1], newNode[0]])
                print "newNode_pos:", newNode_pos
                newNode_small = pos_2_map_index(map_msg, MAP_RES_ASTAR, newNode_pos)
                print "newNode_small:", newNode_small
                print "newNode_pos after conversion:", map_index_2_pos(map_msg, MAP_RES_ASTAR, newNode_small)
                walkingNode_pos = map_index_2_pos(map_msg, MAP_RES_COVERING, [walkingNode[1], walkingNode[0]])
                walkingNode_small = pos_2_map_index(map_msg, MAP_RES_ASTAR, walkingNode_pos)
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
                    print "after: walkingNode_pos after conversion:", map_index_2_pos(map_msg, MAP_RES_ASTAR, walkingNode_small)
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
                    print "after: newNode_pos after conversion:", map_index_2_pos(map_msg, MAP_RES_ASTAR, newNode_small)
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


        raw_path = [np.array(pathX), np.array(pathY)]

        pathX=np.fliplr([pathX])[0]
        pathY=np.fliplr([pathY])[0]
        xRoute=[]
        yRoute=[]


        dir=0
        for pathCount in range(0,len(pathX)-1):

            prevX=pathX[pathCount+1]
            prevY=pathY[pathCount+1]


            if prevX==pathX[pathCount]:
                if pathY[pathCount]>prevY:
                    if dir!=5:
                        xRoute.append(pathX[pathCount])
                        yRoute.append(pathY[pathCount])
                        dir=5
                else:
                    if dir!=4:
                        xRoute.append(pathX[pathCount])
                        yRoute.append(pathY[pathCount])
                        dir=4
            elif prevY==pathY[pathCount]:
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

            elif (prevX==(pathX[pathCount]+1) or prevX==(pathX[pathCount]+2) ) and (prevY==(pathY[pathCount]+1) or prevY==(pathY[pathCount]+2)) or (prevX==(pathX[pathCount]-1) or prevX==(pathX[pathCount]-2)) and (prevY==(pathY[pathCount]-1) or prevY==(pathY[pathCount]-2)):
                if ((pathX[pathCount]+1) or prevX==(pathX[pathCount]+2) ) and (prevY==(pathY[pathCount]+1) or prevY==(pathY[pathCount]+1)):
                    if dir!=2:
                        xRoute.append(pathX[pathCount])
                        yRoute.append(pathY[pathCount])
                        dir=2
                else:
                    if dir!=22:
                        xRoute.append(pathX[pathCount])
                        yRoute.append(pathY[pathCount])
                        dir=22
            elif (prevX==(pathX[pathCount]+1) or prevX==(pathX[pathCount]+2) ) and (prevY==(pathY[pathCount]-1) or prevY==(pathY[pathCount]-2)) or (prevX==(pathX[pathCount]-1) or prevX==(pathX[pathCount]-2)) and (prevY==(pathY[pathCount]+1) or prevY==(pathY[pathCount]+2)):
                if ((pathX[pathCount]+1) or prevX==(pathX[pathCount]+2)) and (prevY==(pathY[pathCount]-1) or prevY==(pathY[pathCount]-2)):
                    if dir!=-2:
                        xRoute.append(pathX[pathCount])
                        yRoute.append(pathY[pathCount])
                        dir=-2
                else:
                    if dir!=-19:
                        xRoute.append(pathX[pathCount])
                        yRoute.append(pathY[pathCount])
                        dir=-19
            else:
                if dir!=-44:
                    xRoute.append(pathX[pathCount])
                    yRoute.append(pathY[pathCount])
                    dir=-44


            #wholeRoute = [wholeXroute, wholeYroute]
        route=[xRoute, yRoute] ############################################################################################################################### before: route=[xRoute, xRoute]
        routeS2G=np.fliplr(route)

        return routeS2G, raw_path

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
