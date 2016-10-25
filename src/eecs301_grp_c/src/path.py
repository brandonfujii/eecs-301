from map import *
from Queue import *
import operator

DIRECTIONS = ['North', 'East', 'South', 'West']

def getPath(start, start_heading, end, end_heading):
    my_map = EECSMap()
    my_map.clearCostMap()
    mapQueue = Queue()
    mapQueue.put(start)
    current_position = start
    current_cost = 0
    my_map.costMap[current_position[0]][current_position[1]] = 0
    visited = []
    visited.append(current_position)
    print my_map.costMap
    while mapQueue.empty() == False:
        current_position = mapQueue.get()
        current_cost = my_map.costMap[current_position[0]][current_position[1]]
        if my_map.getNeighborObstacle(current_position[0], current_position[1], DIRECTION.North) == 0 and ([current_position[0]-1, current_position[1]] in visited) == False:
            next_position = [current_position[0]-1, current_position[1]]
            mapQueue.put(next_position)
            visited.append(next_position)
            my_map.costMap[current_position[0]-1][current_position[1]] = current_cost+1
        if my_map.getNeighborObstacle(current_position[0], current_position[1], DIRECTION.East) == 0 and ([current_position[0], current_position[1]+1] in visited) == False:
            next_position = [current_position[0], current_position[1]+1]
            mapQueue.put(next_position)
            visited.append(next_position)
            my_map.costMap[current_position[0]][current_position[1]+1] = current_cost+1
        if my_map.getNeighborObstacle(current_position[0], current_position[1], DIRECTION.South) == 0 and ([current_position[0]+1, current_position[1]] in visited) == False:
            next_position = [current_position[0]+1, current_position[1]]
            mapQueue.put(next_position)
            visited.append(next_position)
            my_map.costMap[current_position[0]+1][current_position[1]] = current_cost+1
        if my_map.getNeighborObstacle(current_position[0], current_position[1], DIRECTION.West) == 0 and ([current_position[0], current_position[1]-1] in visited) == False:
            next_position = [current_position[0], current_position[1]-1]
            mapQueue.put(next_position)
            visited.append(next_position)
            my_map.costMap[current_position[0]][current_position[1]-1] = current_cost+1

    my_map.printCostMap()
    my_map.printObstacleMap()


    current_position = end
    last_position = end
    current_heading = start_heading
    directions = []
    positions = []
    positions.append(end)

    while current_position != start:
        neighbor_costs = []
        if my_map.getNeighborObstacle(current_position[0], current_position[1], DIRECTION.North) == 0:
            neighbor_costs.append(my_map.getNeighborCost(current_position[0],current_position[1], DIRECTION.North))
        if my_map.getNeighborObstacle(current_position[0], current_position[1], DIRECTION.East) == 0:
            neighbor_costs.append(my_map.getNeighborCost(current_position[0],current_position[1], DIRECTION.East))
        if my_map.getNeighborObstacle(current_position[0], current_position[1], DIRECTION.South) == 0:
            neighbor_costs.append(my_map.getNeighborCost(current_position[0],current_position[1], DIRECTION.South))
        if my_map.getNeighborObstacle(current_position[0], current_position[1], DIRECTION.West) == 0:
            neighbor_costs.append(my_map.getNeighborCost(current_position[0],current_position[1], DIRECTION.West))

        minimum_neighbor_cost = min(neighbor_costs)

        if minimum_neighbor_cost == my_map.getNeighborCost(current_position[0],current_position[1], DIRECTION.North):

            current_position = [current_position[0]-1, current_position[1]]
            positions.append(current_position)
            directions.append(list(map(operator.sub, current_position, last_position)))
            last_position = current_position

        elif minimum_neighbor_cost == my_map.getNeighborCost(current_position[0],current_position[1], DIRECTION.East):

            current_position = [current_position[0], current_position[1]+1]
            positions.append(current_position)
            directions.append(list(map(operator.sub, current_position, last_position)))
            last_position = current_position

        elif minimum_neighbor_cost == my_map.getNeighborCost(current_position[0],current_position[1], DIRECTION.South):

            current_position = [current_position[0]+1, current_position[1]]
            positions.append(current_position)
            directions.append(list(map(operator.sub, current_position, last_position)))
            last_position = current_position

        elif minimum_neighbor_cost == my_map.getNeighborCost(current_position[0],current_position[1], DIRECTION.West):

            current_position = [current_position[0], current_position[1]-1]
            positions.append(current_position)
            directions.append(list(map(operator.sub, current_position, last_position)))
            last_position = current_position

    print "Path from %s at %s to %s at %s" % (start, DIRECTIONS[start_heading - 1], end, DIRECTIONS[end_heading - 1])
    steps = []
    for i in range(0,len(directions)):
        if ((directions[-i-1] == [-1,0] and current_heading == 3) or
            (directions[-i-1] == [0,-1] and current_heading == 2) or
            (directions[-i-1] == [1, 0] and current_heading == 1) or
            (directions[-i-1] == [0, 1] and current_heading == 4) ):
            steps.append("Go Forward")
            # Keep current heading
        elif ((directions[-i-1] == [-1,0] and current_heading == 4) or
            (directions[-i-1] == [0,-1] and current_heading == 3) or
            (directions[-i-1] == [1, 0] and current_heading == 2) or
            (directions[-i-1] == [0, 1] and current_heading == 1) ):
            steps.append("Go Left")
            current_heading = (current_heading + 3) % 4
        elif ((directions[-i-1] == [-1,0] and current_heading == 1) or
            (directions[-i-1] == [0,-1] and current_heading == 4) or
            (directions[-i-1] == [1, 0] and current_heading == 3) or
            (directions[-i-1] == [0, 1] and current_heading == 2) ):
            steps.append("Go Backward")
            current_heading = (current_heading + 2) % 4 # reverse heading
        elif ((directions[-i-1] == [-1,0] and current_heading == 2) or
              (directions[-i-1] == [0,-1] and current_heading == 1) or
              (directions[-i-1] == [1, 0] and current_heading == 4) or
              (directions[-i-1] == [0, 1] and current_heading == 3) ):
            steps.append("Go Right")
            current_heading = (current_heading + 1)%4

        # Correct heading
        if current_heading > 4:
            current_heading = current_heading - 4
        elif current_heading < 1:
            current_heading = current_heading + 4

    # if (current_heading-end_heading)==1 or (current_heading-end_heading)==-3:
    #     steps.append("Turn left")
    # elif (current_heading-end_heading)==-1 or (current_heading-end_heading)==3:
    #     steps.append("Turn right")
    # elif (current_heading-end_heading)==2 or (current_heading-end_heading)==-2:
    #     steps.append("Turn around")
        
    print steps
    return steps

getPath([7, 6], 2, [0, 6], 1)
