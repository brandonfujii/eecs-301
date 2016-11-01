#!/usr/bin/env python
import roslib
import signal
import rospy
import sys
from fw_wrapper.srv import *
import operator
from map import *
from path import *

# -----------SERVICE DEFINITION-----------
# allcmd REQUEST DATA
# ---------
# string command_type
# int8 device_id
# int16 target_val
# int8 n_dev
# int8[] dev_ids
# int16[] target_vals

# allcmd RESPONSE DATA
# ---------
# int16 val
# --------END SERVICE DEFINITION----------

# ----------COMMAND TYPE LIST-------------
# GetMotorTargetPosition
# GetMotorCurrentPosition
# GetIsMotorMoving
# GetSensorValue
# GetMotorWheelSpeed
# SetMotorTargetPosition
# SetMotorTargetSpeed
# SetMotorTargetPositionsSync
# SetMotorMode
# SetMotorWheelSpeed

MIN_SHOULDER_POS = 280
MAX_SHOULDER_POS = 744
MIN_ELBOW_POS = 210
MAX_ELBOW_POS = 814
IR_PORT = 2
HEAD_PORT = 1
INNER_OFFSET = 200
OUTER_OFFSET = 75
PORT_MAP = {'back_right_wheel': 16, 'back_left_wheel' : 12, 'front_right_wheel': 11, 'front_left_wheel': 9 }

# wrapper function to call service to set a motor mode
# 0 = set target positions, 1 = set wheel moving
def setMotorMode(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorMode', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get motor wheel speed
def getMotorWheelSpeed(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorWheelSpeed', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor wheel speed
def setMotorWheelSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorWheelSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor target speed
def setMotorTargetSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get sensor value
def getSensorValue(port):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetSensorValue', port, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set a motor target position
def setMotorTargetPositionCommand(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetPosition', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get a motor's current position
def getMotorPositionCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorCurrentPosition', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to check if a motor is currently moving
def getIsMotorMovingCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetIsMotorMoving', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

class Leg:
	def __init__(self, wheel_port):
		self.wheel = wheel_port


	def setWheelSpeed(self, direction, speed):
		if speed > 1024 or speed < 0:
			rospy.loginfo("Speed must be between 0 and 1024")
		elif direction == 'forward':
			if self.wheel == PORT_MAP['back_right_wheel'] or self.wheel == PORT_MAP['front_right_wheel']:
				setMotorWheelSpeed(self.wheel, 1024 + speed)
			else:
				setMotorWheelSpeed(self.wheel, speed)
		elif direction == 'backward':
			if self.wheel == PORT_MAP['back_right_wheel'] or self.wheel == PORT_MAP['front_right_wheel']:
				setMotorWheelSpeed(self.wheel, speed)
			else:
				setMotorWheelSpeed(self.wheel, 1024 + speed)
		else:
			rospy.loginfo("Direction must be forward, or backward")

class Robot:
	def __init__(self):
		self.left_ir_port = 3
		self.right_ir_port = 6
		self.head_port = 4
		self.backRightLeg = Leg(PORT_MAP['back_right_wheel'])
		self.backLeftLeg = Leg(PORT_MAP['back_left_wheel'])
		self.frontRightLeg = Leg(PORT_MAP['front_right_wheel'])
		self.frontLeftLeg = Leg(PORT_MAP['front_left_wheel'])
		self.head_threshold = 1100
		self.left_threshold = 200
		self.right_threshold = 100
		self.left_wall_threshold = 480
		self.right_wall_threshold = 340
		self.action = None


	def setAllWheels(self, direction, speed):
		self.backLeftLeg.setWheelSpeed(direction, speed)
		self.frontLeftLeg.setWheelSpeed(direction, speed)
		self.backRightLeg.setWheelSpeed(direction, speed)
		self.frontRightLeg.setWheelSpeed(direction, speed)

	def drive(self):
	    offset = 30
	    self.backLeftLeg.setWheelSpeed('forward', 1000-offset)
	    self.frontLeftLeg.setWheelSpeed('forward', 1000-offset)
	    self.backRightLeg.setWheelSpeed('forward', 1000)
	    self.frontRightLeg.setWheelSpeed('forward', 1000)
	    
	def stop(self):
	    self.setAllWheels('forward', 0)

	def turnWheelRight(self, speed = 1000):
	    self.backLeftLeg.setWheelSpeed('forward', speed)
	    self.frontLeftLeg.setWheelSpeed('forward', speed)
	    self.backRightLeg.setWheelSpeed('backward', speed)
	    self.frontRightLeg.setWheelSpeed('backward', speed)

	def turnWheelLeft(self, speed = 1000):
	    self.backLeftLeg.setWheelSpeed('backward', speed)
	    self.frontLeftLeg.setWheelSpeed('backward', speed)
	    self.backRightLeg.setWheelSpeed('forward', speed)
	    self.frontRightLeg.setWheelSpeed('forward', speed)

	def turnRight_90(self):
		self.turnWheelRight()
		wait(0.70)
		self.stop()
		wait(1)

	def turnLeft_90(self):
	    self.turnWheelLeft()
	    wait(0.70)
	    self.stop()
	    wait(1)

	def turnAround(self):
		self.turnWheelRight()
		wait(1.4)
		self.stop()
		wait(.1)
	    
	def straight(self, num_squares):
	    self.turnWheelLeft(175)
	    wait(0.1)
	    self.drive()
	    if num_squares == 1:
	        wait(1.6)
	    else:
	        wait(1.7*num_squares)
	    self.turnWheelRight(200)
	    wait(0.2)
	    self.stop()
	    wait(1)
	    
	    
	def follow_instructions(self, instr_array):
		num_forwards = 0
	    for instr in instr_array:
	        if instr == 'Go Forward':
	        	num_forwards +=1
	        else:
	        	if num_forwards > 0:
	        		self.straight(num_forwards)
	        		num_forwards = 0

	        	if instr == 'Turn Left':
	        		self.turnLeft_90()
	        	elif instr == 'Turn Right':
	        		self.turnRight_90()
	        	elif instr == 'Turn Around':
	        		self.turnAround()


"""
	def straight(self, number_of_moves, wheelState):
	    self.driving_position
	    prev_position = wheelState[0]
	    total_dist = wheelState[1]
	    
	    if total_dist < 1024 * 2.9 * number_of_moves:
	        self.drive()
	        position = getMotorPositionCommand(11)
	        rospy.loginfo(position)
	        rospy.loginfo(prev_position)
	        rospy.loginfo("----")
	        
	        if position >= prev_position:
	            new_dist = position - prev_position
	        else:
	            new_dist = 1024 - prev_position + position
	         
		    total_dist += new_dist
		    
		    rospy.loginfo(new_dist)
		    rospy.loginfo(total_dist)
		    rospy.loginfo("-----------")
		    
		    prev_position = position
		#else:
		#    self.stop()
		
		wheelState = [prev_position, total_dist]
		return wheelState
"""
		    

"""
	def move_east(self, number_of_moves):
	    self.turnRight_90()
	    for i in xrange(0, number_of_moves):
	        self.north()
	    #self.straight(number_of_moves)
	    self.neutral_position()
	    self.turnRight(100)


	def west(self):
	    self.turnLeft_90()
	    for i in xrange(0, 3):
	        self.walk2()

	def followWall(self, wall, number_of_steps):
	    if wall == 'right':
	        right_sensor_value = getSensorValue(self.right_ir_port)
	        rospy.loginfo(number_of_steps)
	        if number_of_steps >= 6:
	            self.turnLeft()
	            return 0
	        elif number_of_steps <= -6:
	            self.turnRight()
	            return 0
	        else:
	            if right_sensor_value >= self.right_wall_threshold:
	               self.stepLeft()
	               return number_of_steps + 1
	            else:
	               self.stepRight()
	               return number_of_steps - 1
	    else:
	        left_sensor_value = getSensorValue(self.left_ir_port)
	        rospy.loginfo(number_of_steps)
	        if number_of_steps >= 10:
	            self.turnLeft()
	            return 0
	        elif number_of_steps <= -7:
	            self.turnLeft()
	            return 0
	        else:
	            if left_sensor_value >= self.left_wall_threshold:
	                rospy.loginfo("Right")
	                self.stepRight()
	                return number_of_steps + 1
	            else:
	                rospy.loginfo("left")
	                self.stepLeft()
	                return number_of_steps - 1
	     """
	"""

		def stepLeft(self):
	    self.action = 'step_left'


	def stepRight(self):
	    self.action = 'step_right'

	def walk2(self):
	    if getSensorValue(self.head_port) >= self.head_threshold:
	        if getSensorValue(self.left_ir_port) >= self.left_threshold and getSensorValue(self.right_ir_port) >= self.right_threshold:
	            self.turnAround()
	        elif getSensorValue(self.right_ir_port) >= self.right_threshold:
	            if getSensorValue(self.left_ir_port) < self.left_threshold:
	                self.turnLeft_90()
	            else:
	                self.turnAround()
	        elif getSensorValue(self.left_ir_port) >= self.left_threshold:
	            if getSensorValue(self.right_ir_port) < self.right_threshold:
	                self.turnRight_90()
	            else:
	                self.turnAround()
	    else:
	        self.stepLeft()
	        self.stepRight()
	        rospy.loginfo("walked")
	        """


def wait(seconds):
    initial = rospy.Time.now()
    while rospy.Time.now() < initial + rospy.Duration(seconds):
        continue

def shutdown(sig, stackframe):
    rospy.loginfo("Setting wheels to zero")
    for wheel in [16, 11, 12, 9]:
        setMotorWheelSpeed(wheel, 0)
    sys.exit(0)
    
def setMotorSpeeds(speed):
    setMotorTargetSpeed(PORT_MAP['back_right_shoulder'], speed)
    setMotorTargetSpeed(PORT_MAP['front_right_shoulder'], speed)
    setMotorTargetSpeed(PORT_MAP['back_left_shoulder'], speed)
    setMotorTargetSpeed(PORT_MAP['front_left_shoulder'], speed)
    
# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")

    signal.signal(signal.SIGINT, shutdown)

    # control loop running at 10hz
    r = rospy.Rate(10) # 10hz
    #for x in xrange(1, 9):
    #   setMotorTargetSpeed(x, 500)

    setMotorSpeeds(50)
    for wheel in [11, 9, 16, 12]:
        setMotorMode(wheel, 1)

    Ross = Robot()
    # Ross.walking_position_left()
    # Ross.turning_position()

    args = sys.argv[1:]
    if not args or len(args) < 6:
        rospy.loginfo("Usage: rosrun eecs301_grp_c asn2.py start_x start_y start_heading end_x end_y end_heading")
        sys.exit(1)
    print args
    start = [int(args[0]), int(args[1])]
    start_heading = int(args[2])
    end = [int(args[3]), int(args[4])]
    end_heading = int(args[5])
    # end_heading = int(args[3])
    # print start, start_heading


    #while not rospy.is_shutdown():
    #    Ross.turnRight_90()
       #wheelState = Ross.straight(1, wheelState)
    # print start, end
        # Ross.drive()
    
    Ross.follow_instructions(getPath(start, start_heading, end, end_heading))
    

    # Ross.neutral_position()
