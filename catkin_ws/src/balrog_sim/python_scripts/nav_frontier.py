import numpy as np

def frontier(slamMap, currentPosition):
    rows, cols = slamMap.shape
    frontierMap = np.zeros(shape=(rows,cols))
    slamMapNoObstacles = slamMap
    slamMapNoObstacles[slamMapNoObstacles == -2] = 0    #Covered
    slamMapNoObstacles[slamMapNoObstacles == 100] = 0   #Obstacle]
    tempMapA = np.vstack( [ np.zeros(shape=(1,rows)), slamMapNoObstacles ] )
    tempMap1 = np.column_stack( [ np.zeros(shape=(cols+1,1)), tempMapA ] )
    tempMapB = np.vstack( [ slamMapNoObstacles, np.zeros(shape=(1,rows)) ] )
    tempMap2 = np.column_stack( [ tempMapB, np.zeros(shape=(cols+1,1)) ] )
    frontierNodes = tempMap2 - tempMap1 ## 16x16
    FirstMap = frontierNodes[0:-1,0:-1]
    SecondMap = frontierNodes[1:,1:]

    frontierMap[FirstMap == -1] = 1
    frontierMap[SecondMap == 1] = 1
    tempMapC = np.vstack( [ slamMapNoObstacles, np.zeros(shape=(1,cols)) ] )
    tempMap3 = np.column_stack( [ np.zeros(shape=(cols+1,1)), tempMapC ] )
    tempMapD = np.column_stack( [ slamMapNoObstacles, np.zeros(shape=(rows,1)) ] )
    tempMap4 = np.vstack( [ np.zeros(shape=(1,cols+1)), tempMapD  ] )
    frontierNodes = tempMap3 - tempMap4

    FirstMap = frontierNodes[:-1,1:]
    SecondMap = frontierNodes[1:,0:-1]
    frontierMap[FirstMap == -1] = 1
    frontierMap[SecondMap == 1] = 1
    #Remove covered area and obstacles as frontier candidates
    frontierMap[slamMap == 100] = 0
    frontierMap[slamMap == -2] = 0
    x = np.nonzero(frontierMap)
    goalNode = np.argmin(((x[0]-currentPosition[0])**2 + (x[1]-currentPosition[1])**2)**0.5)

    return [x[0][goalNode],x[1][goalNode]]

if __name__ == "__main__":
    test = 1
