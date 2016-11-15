import numpy as np

def update_pos(pos, heading):
    if heading == 1:
        return [ pos[0]-1, pos[1] ]
    elif heading == 2:
        return [ pos[0], pos[1]+1 ]
    elif heading == 3:
        return [ pos[0]+1, pos[1]-1 ]
    elif heading == 4:
        return [ pos[0], pos[1]-1 ]

def change_heading(heading):
    if heading > 4:
        heading = heading - 4
    elif heading < 1:
        heading = heading + 4

    return heading


def explored(dir, heading, pos, visited):
    # gives back either a 1 or a 0
    if dir == 'forward':
        # check if position is possible in that direction
        if heading == 1 and pos[0] - 1 >= 0:
            return visited[pos[0] - 1, pos[1]]
        elif heading == 2 and pos[1] + 1 <= 7:
            return visited[pos[0], pos[1]+1]
        elif heading == 3 and pos[0] + 1 <= 7:
            return visited[pos[0] + 1, pos[1]]
        elif heading == 4 and pos[1] - 1 >= 0:
            return visited[pos[0], pos[1] - 1]
    elif dir == 'left':
        if heading == 1 and pos[1]-1 >= 0:
            return visited[pos[0], pos[1]-1]
        elif heading == 2 and pos[0]-1 >= 0:
            return visited[pos[0]-1, pos[1]]
        elif (heading == 3) and (pos[1]+1 <= 7):
            return visited[pos[0], pos[1]+1]
        elif heading == 4 and pos[0]+1 <= 7:
            return visited[pos[0]+1, pos[1]]
    elif dir == 'right':
         if heading == 1 and pos[1]+1 <= 7:
            return visited[pos[0], pos[1]+1]
        elif heading == 2 and pos[0]+1 <= 7:
            return visited[pos[0]+1, pos[1]]
        elif heading == 3 and pos[1]-1 >= 0:
            return visited[pos[0], pos[1]-1]
        elif heading == 4 and pos[0]-1 >= 0:
            return visited[pos[0]-1, pos[1]]
    else:
        raise Exception('Does not recognize this direction')

def main():

    my_map = EECSMap()
    head_sensor = getSensorValue(Ross.head_port)
    left_sensor = getSensorValue(Ross.left_ir_port)
    right_sensor = getSensorValue(Ross.right_ir_port)
    visited = np.zeros((8,8))
    pos = [0, 0]
    heading = 3
    visited[pos[0], pos[1]] = 1

    while not rospy.is_shutdown():
        # Front is clear, sides are blocked
        if (head_sensor < DMSThreshold) and (right_sensor > 0) and (left_sensor > 0):
            # Just drive forward
            my_map.setObstacle(pos[0], pos[1], 0, heading)
            pos = update_pos(pos, heading)

        # Front and left are blocked, right is clear
        elif (head_sensor >= DMSThreshold) and (right_sensor == 0) and (left_sensor > 0):
            # Turn right and drive one tile
            next_heading = change_heading((heading+1) % 4)
            my_map.setObstacle(pos[0], pos[1], 0, next_heading)
            pos = update_pos(pos, next_heading)

        # Front and right are blocked, left is clear
        elif (head_sensor >= DMSThreshold) and (right_sensor > 0) and (left_sensor == 0):
            # Turn left and drive one tile
            next_heading = change_heading((heading + 3) % 4)
            my_map.setObstacle(pos[0], pos[1], 0, )
            pos = update_pos(pos, next_heading)

        # if both front and right are clear and both are unexplored
        elif (head_sensor < DMSThreshold) and (right_sensor == 0) and (left_sensor > 0) and (not explored("right", heading, pos, visited)) and (not explored("forward", heading, pos, visited)):


        # if front and left are clear and both are unexplored
        elif (head_sensor < DMSThreshold) and (right_sensor > 0) and (left_sensor == 0) and (not explored("left", heading, pos, visited)) and (not explored("forward", heading, pos, visited)):

        # if right and left are clear and both are unexplored
        elif (head_sensor >= DMSThreshold) and (right_sensor == 0) and (left_sensor == 0) and (not explored("right", heading, pos, visited)) and (not explored("left", heading, pos, visited)):
            # Turn Right and Forward

        # if all sides are clear and all are unexplored
        elif (head_sensor < DMSThreshold) and (right_sensor == 0) and (left_sensor == 0) and (not explored("right", heading, pos, visited)) and (not explored("left", heading, pos, visited)) and (not explored("forward", heading, pos, visited)):
            # Go Forward

        # if front and right are clear and right is explored and front is unexplored
        elif (head_sensor < DMSThreshold) and (right_sensor == 0) and (left_sensor > 0) and (explored("right", heading, pos, visited)) and (not explored("forward", heading, pos, visited)):
            # Go Forward

        # If front and right are clear and right is unexplored and front is explored
        elif (head_sensor < DMSThreshold) and (right_sensor == 0) and (left_sensor > 0) and (not explored("right", heading, pos, visited)) and (explored("forward", heading, pos, visited)):
            # Go Right

        # If front and left are clear and left is explored and front is unexplored
        elif (head_sensor < DMSThreshold) and (right_sensor > 0) and (left_sensor == 0) and (explored("left", heading, pos, visited)) and (not explored("forward", heading, pos, visited)):
            # Go forward

        # If front and left are clear and left is unexplored and front is explored
        elif (head_sensor < DMSThreshold) and (right_sensor > 0) and (left_sensor == 0) and (not explored("left", heading, pos, visited)) and (explored("forward", heading, pos, visited)):
            # Go left

        # If right and left are clear and left is unexplored and right is explored
        elif (head_sensor >= DMSThreshold) and (right_sensor == 0) and (left_sensor == 0) and (not explored("left", heading, pos, visited)) and (explored("right", heading, pos, visited)):
            # Go left

        # If right and left are clear and left is explored and right is unexplored
        elif (head_sensor >= DMSThreshold) and (right_sensor == 0) and (left_sensor == 0) and (explored("left", heading, pos, visited)) and (not explored("right", heading, pos, visited)):
            # Go right
