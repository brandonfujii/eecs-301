from map import *
from Queue import *

def single_path(my_map, start, start_heading, end, end_heading):
    initial_position = start
    map_queue = Queue()
    my_map.clearCostMap()
    current_position = start
    current_cost = 0
    map_queue.put(current_position)
    my_map.costMap[current_position[0]][current_position[1]] = 0
    visited = []
    visited.append(current_position)
    while not map_queue.empty():
        current_position = map_queue.get()
        current_cost = my_map.costMap[current_position[0]][current_position[1]]
        if my_map.getNeighborObstacle(current_position[0], current_position[1], DIRECTION.North) == 0 and ([current_position[0]- 1 , current_position[1]] not in visited):
            next_position = [current_position[0] - 1, current_position[1]]
            map_queue.put(next_position)
            visited.append(next_position)
            my_map.costMap[current_position[0] - 1][current_position[1]] = current_cost + 1
        if my_map.getNeighborObstacle(current_position[0], current_position[1], DIRECTION.South) == 0 and ([current_position[0] + 1, current_position[1]] not in visited):
            next_position = [current_position[0] + 1, current_position[1]]
            map_queue.put(next_position)
            visited.append(next_position)
            my_map.costMap[current_position[0] + 1][current_position[1]] = current_cost + 1
        if my_map.getNeighborObstacle(current_position[0], current_position[1], DIRECTION.East) == 0 and ([current_position[0], current_position[1] + 1] not in visited):
            next_position = [current_position[0], current_position[1] + 1]
            map_queue.put(next_position)
            visited.append(next_position)
            my_map.costMap[current_position[0]][current_position[1] + 1] = current_cost + 1
        if my_map.getNeighborObstacle(current_position[0], current_position[1] - 1, DIRECTION.West) == 0 and ([current_position[0], current_position[1] - 1] not in visited):
            next_position = [current_position[0], current_position[1] - 1]
            map_queue.put(next_position)
            visited.append(next_position)
            my_map.costMap[current_position[0]][current_position[1] - 1] = current_cost + 1
    my_map.printCostMap()
    my_map.printObstacleMap()
def main():
    my_map = EECSMap()
    single_path(my_map, [0, 0], 0, [7, 7], 3)

if __name__ == "__main__":
    main()
