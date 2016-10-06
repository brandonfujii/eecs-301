#!/usr/bin/env python
import roslib
import rospy
from fw_wrapper.srv import *
import operator

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
PORT_MAP = { 'back_right_shoulder': 4, 'back_right_elbow': 8, 'back_left_shoulder': 3, 'back_left_elbow' : 7, 'front_right_shoulder': 2, 'front_right_elbow': 6, 'front_left_shoulder': 1, 'front_left_elbow': 5 }

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
	def __init__(self, elbow_port, shoulder_port, up, forward, max_forward, max_back, max_up, max_down):
		self.elbow = elbow_port
		self.shoulder = shoulder_port
		self.up = up
		self.forward = forward
		self.max_forward = max_forward
		self.max_back = max_back
		self.max_up = max_up
		self.max_down = max_down

	def getPosition(self, part):
		if part == 'elbow':
			return getMotorPositionCommand(self.elbow)
		elif part == 'shoulder':
			return getMotorPositionCommand(self.shoulder)
		else:
			rospy.loginfo('Please pass in an elbow or a shoulder')

	def setPosition(self, part, target_val, offset = 0):
		if part == 'elbow':
		    setMotorTargetPositionCommand(self.elbow, self.up(target_val, offset))
		elif part == 'shoulder':
		    setMotorTargetPositionCommand(self.shoulder, self.forward(target_val, offset))
		else:
			rospy.loginfo('Please pass in an elbow or a shoulder')

	def moveLeg(self, shoulder_pos, elbow_pos, speed = 200):
		setMotorTargetSpeed(self.elbow, speed)
		setMotorTargetSpeed(self.shoulder, speed)
		self.setPosition('shoulder', shoulder_pos)
		self.setPosition('elbow', elbow_pos)
		
class Robot:
	def __init__(self):
		self.left_ir_port = 2
		self.right_ir_port = 5
		self.head_port = 1
		self.backRightLeg = Leg(8, 4, operator.add, operator.add, 544, 205, 814, 210)
		self.backLeftLeg = Leg(7, 3, operator.sub, operator.sub, 480, 819, 210, 814)
		self.frontRightLeg = Leg(6, 2, operator.sub, operator.add, 819, 480, 210, 814)
		self.frontLeftLeg = Leg(5, 1, operator.add, operator.sub, 205, 544, 814, 210)
		self.states = [
		    { 
		      'back_right_shoulder' : self.backRightLeg.max_forward,
		      'back_right_elbow' : self.backRightLeg.max_down,
		      'back_left_shoulder' : self.backLeftLeg.max_back,
		      'back_left_elbow' : self.backLeftLeg.max_down,
		      'front_right_shoulder' : self.frontRightLeg.max_back,
		      'front_right_elbow' : self.frontRightLeg.max_down,
		      'front_left_shoulder' : self.frontLeftLeg.max_forward,
		      'front_left_elbow' : self.frontLeftLeg.max_down
		    },
		    { 
		      'back_right_shoulder' : self.backRightLeg.max_forward,
		      'back_right_elbow' : self.backRightLeg.max_down,
		      'back_left_shoulder' : self.backLeftLeg.max_back,
		      'back_left_elbow' : self.backLeftLeg.max_down,
		      'front_right_shoulder' : self.frontRightLeg.forward(self.frontRightLeg.max_forward, -150),
		      'front_right_elbow' : self.frontRightLeg.up(self.frontRightLeg.max_down, 50),
		      'front_left_shoulder' : self.frontLeftLeg.max_forward,
		      'front_left_elbow' : self.frontLeftLeg.max_down
		    },
		    { 
		      'back_right_shoulder' : self.backRightLeg.max_forward,
		      'back_right_elbow' : self.backRightLeg.max_down,
		      'back_left_shoulder' : self.backLeftLeg.max_back,
		      'back_left_elbow' : self.backLeftLeg.max_down,
		      'front_right_shoulder' : self.frontRightLeg.forward(self.frontRightLeg.max_forward, -150),
		      'front_right_elbow' : self.frontRightLeg.up(self.frontRightLeg.max_down, 50),
		      'front_left_shoulder' : self.frontLeftLeg.max_forward,
		      'front_left_elbow' : self.frontLeftLeg.max_down
		    },
		    {
		      'back_right_shoulder' : self.backRightLeg.max_back,
		      'back_right_elbow' : self.backRightLeg.max_down,
		      'back_left_shoulder' : self.backLeftLeg.max_forward,
		      'back_left_elbow' : self.backLeftLeg.max_down,
		      'front_right_shoulder' : self.frontRightLeg.max_forward,
		      'front_right_elbow' : self.frontRightLeg.max_down,
		      'front_left_shoulder' : self.frontLeftLeg.max_back,
		      'front_left_elbow' : self.frontLeftLeg.max_down
		    }
		    
		]

		self.turningRightSteps = [
			{
				'back_right_shoulder' : self.backRightLeg.forward(self.backRightLeg.max_back, 200),
				'back_right_elbow' : self.backRightLeg.max_down,
				'back_left_shoulder' : self.backLeftLeg.forward(self.backLeftLeg.max_forward, -200),
				'back_left_elbow' : self.backLeftLeg.max_down,
				'front_right_shoulder' : self.frontRightLeg.forward(self.frontRightLeg.max_back, 200),
				'front_right_elbow' : self.frontRightLeg.max_down,
				'front_left_shoulder' : self.frontLeftLeg.forward(self.frontLeftLeg.max_forward, -200),
				'front_left_elbow' : self.frontLeftLeg.max_down
			},
			{
				'front_right_elbow' : self.frontRightLeg.up(self.frontRightLeg.max_down, 50),
				'front_right_shoulder' : self.frontRightLeg.max_back
			},
			{
				'front_right_elbow' : self.frontRightLeg.max_down
			},
			{
				'back_right_elbow' : self.backRightLeg.up(self.backRightLeg.max_down, 50),
				'back_right_shoulder' : self.backRightLeg.max_back
			},
			{
				'back_right_elbow' : self.backRightLeg.max_down
			},
			{
				'back_left_elbow' : self.backLeftLeg.up(self.backLeftLeg.max_down, 50),
				'back_left_shoulder' : self.backLeftLeg.max_forward
			},
			{
				'back_left_elbow' : self.backLeftLeg.max_down
			},
			{
				'front_left_elbow' : self.frontLeftLeg.up(self.frontLeftLeg.max_down, 50),
				'front_left_shoulder' : self.frontLeftLeg.max_forward
			},
			{
				'front_left_elbow' : self.frontLeftLeg.max_down
			}
		]

	def getLegPositions(self):
		leg_positions = {}
		leg_positions['back_right_shoulder'] = getMotorPositionCommand(self.backRightLeg.shoulder)
		leg_positions['back_right_elbow'] = getMotorPositionCommand(self.backRightLeg.elbow)
		leg_positions['back_left_shoulder'] = getMotorPositionCommand(self.backLeftLeg.shoulder)
		leg_positions['back_left_elbow'] = getMotorPositionCommand(self.backLeftLeg.elbow)
		leg_positions['front_right_shoulder'] = getMotorPositionCommand(self.frontRightLeg.shoulder)
		leg_positions['front_right_elbow'] = getMotorPositionCommand(self.frontRightLeg.elbow)
		leg_positions['front_left_shoulder'] = getMotorPositionCommand(self.frontLeftLeg.shoulder)
		leg_positions['front_left_elbow'] = getMotorPositionCommand(self.frontLeftLeg.elbow)
		return leg_positions
		
	def neutral_position(self):
	    self.backRightLeg.setPosition('elbow', self.backRightLeg.max_down)
	    self.backRightLeg.setPosition('shoulder', self.backRightLeg.forward(self.backRightLeg.max_back, 200))
	    self.backLeftLeg.setPosition('elbow', self.backLeftLeg.max_down)
	    self.backLeftLeg.setPosition('shoulder', self.backLeftLeg.forward(self.backLeftLeg.max_forward, -200))
	    self.frontRightLeg.setPosition('elbow', self.frontRightLeg.max_down)
	    self.frontRightLeg.setPosition('shoulder', self.frontRightLeg.forward(self.frontRightLeg.max_back, 200))
	    self.frontLeftLeg.setPosition('elbow', self.frontLeftLeg.max_down)
	    self.frontLeftLeg.setPosition('shoulder', self.frontLeftLeg.forward(self.frontLeftLeg.max_forward, -200))

	def walking_position_right(self):
		self.backRightLeg.setPosition('elbow', self.backRightLeg.max_down)
		self.backRightLeg.setPosition('shoulder', self.backRightLeg.max_forward)
		self.backLeftLeg.setPosition('elbow', self.backLeftLeg.max_down)
		self.backLeftLeg.setPosition('shoulder', self.backLeftLeg.max_back)
		self.frontRightLeg.setPosition('elbow', self.frontRightLeg.max_down)
		self.frontRightLeg.setPosition('shoulder', self.frontRightLeg.max_back)
		self.frontLeftLeg.setPosition('elbow', self.frontLeftLeg.max_down)
		self.frontLeftLeg.setPosition('shoulder', self.frontLeftLeg.max_forward)
		
	def walking_position_left(self):
	    self.backRightLeg.setPosition('elbow', self.backRightLeg.max_down)
	    self.backRightLeg.setPosition('shoulder', self.backRightLeg.max_back)
	    self.backLeftLeg.setPosition('elbow', self.backLeftLeg.max_down)
	    self.backLeftLeg.setPosition('shoulder', self.backLeftLeg.max_forward)
	    self.frontRightLeg.setPosition('elbow', self.frontRightLeg.max_down)
	    self.frontRightLeg.setPosition('shoulder', self.frontRightLeg.max_forward)
	    self.frontLeftLeg.setPosition('elbow', self.frontLeftLeg.max_down)
	    self.frontLeftLeg.setPosition('shoulder', self.frontLeftLeg.max_back)
	
	def step_1(self):
	    self.frontRightLeg.setPosition('elbow', self.frontRightLeg.up(self.frontRightLeg.max_down, 50))
	    self.frontRightLeg.setPosition('shoulder', self.frontRightLeg.max_forward)
	    
	def back_step_1(self):
	    self.backLeftLeg.setPosition('elbow', self.backLeftLeg.up(self.backLeftLeg.max_down, 50))
	    self.backLeftLeg.setPosition('shoulder', self.backLeftLeg.max_forward)
	
	def back_step_2(self):
	    self.backRightLeg.setPosition('elbow', self.backRightLeg.up(self.backRightLeg.max_down, 50))
	    self.backRightLeg.setPosition('shoulder', self.backRightLeg.max_forward)
	    
	def step_2(self):
	    self.frontLeftLeg.setPosition('elbow', self.frontLeftLeg.up(self.frontLeftLeg.max_down, 50))
	    self.frontLeftLeg.setPosition('shoulder', self.frontLeftLeg.max_forward)
	   
	def walk(self):
	    if getSensorValue(self.head_port) >= 600:
	        rospy.loginfo("head port")
	        self.turnRight()
	        self.neutral_position()
	        wait(0.5)
	
	    self.walking_position_left()
	    wait(0.2)
	    self.step_2()
	    wait(0.1)
	    self.back_step_2()
	    wait(0.1)
	    self.walking_position_right()
	    wait(0.2)
	    self.step_1()
	    wait(0.1)
	    self.back_step_1()
	    wait(0.1)
	    
	def in_turning_right_position(self, step_num)
		TOL = 5
		check_positions = self.turningRightSteps[step_num]
		leg_positions = self.getLegPositions()
		in_position = True
		for motor in check_positions:
			if abs(check_positions[motor] - leg_positions[motor]) > TOL
				in_position = False
				break
		return in_position

	def turnRight(self):
	    self.neutral_position()
	    wait(0.5)
	    self.frontRightLeg.setPosition('elbow', self.frontRightLeg.up(self.frontRightLeg.max_down, 50))
	    self.frontRightLeg.setPosition('shoulder', self.frontRightLeg.max_back)
	    wait(0.1)
	    self.frontRightLeg.setPosition('elbow', self.frontRightLeg.max_down)
	    
	    self.backRightLeg.setPosition('elbow', self.backRightLeg.up(self.backRightLeg.max_down, 50))
	    self.backRightLeg.setPosition('shoulder', self.backRightLeg.max_back)
	    wait(0.1)
	    self.backRightLeg.setPosition('elbow', self.backRightLeg.max_down)
	    
	    self.backLeftLeg.setPosition('elbow', self.backLeftLeg.up(self.backLeftLeg.max_down, 50))
	    self.backLeftLeg.setPosition('shoulder', self.backLeftLeg.max_forward)
	    wait(0.1)
	    self.backLeftLeg.setPosition('elbow', self.backLeftLeg.max_down)
	    
	    self.frontLeftLeg.setPosition('elbow', self.frontLeftLeg.up(self.frontLeftLeg.max_down, 50))
	    self.frontLeftLeg.setPosition('shoulder', self.frontLeftLeg.max_forward)
	    wait(0.1)
	    self.frontLeftLeg.setPosition('elbow', self.frontLeftLeg.max_down)

	def turnRight2(self, turningRightState):
		# update state
		if turningRightState <= 8
			if self.in_turning_right_position(turningRightState)
				turningRightState += 1
					return turningRightState

		# execute step
		if turningRightState == 0
			self.neutral_position()
		elif turningRightState == 1
			self.frontRightLeg.setPosition('elbow', self.frontRightLeg.up(self.frontRightLeg.max_down, 50))
			self.frontRightLeg.setPosition('shoulder', self.frontRightLeg.max_back)
		elif turningRightState == 2
			self.frontRightLeg.setPosition('elbow', self.frontRightLeg.max_down)
		elif turningRightState == 3
			self.backRightLeg.setPosition('elbow', self.backRightLeg.up(self.backRightLeg.max_down, 50))
			self.backRightLeg.setPosition('shoulder', self.backRightLeg.max_back)
		elif turningRightState == 4
			self.backRightLeg.setPosition('elbow', self.backRightLeg.max_down)
		elif turningRightState == 5
			self.backLeftLeg.setPosition('elbow', self.backLeftLeg.up(self.backLeftLeg.max_down, 50))
			self.backLeftLeg.setPosition('shoulder', self.backLeftLeg.max_forward)
	    elif turningRightState == 6
	    	self.backLeftLeg.setPosition('elbow', self.backLeftLeg.max_down)
	    elif turningRightState == 7
	    	self.frontLeftLeg.setPosition('elbow', self.frontLeftLeg.up(self.frontLeftLeg.max_down, 50))
	    	self.frontLeftLeg.setPosition('shoulder', self.frontLeftLeg.max_forward)
	    elif turningRightState == 8
	    	self.frontLeftLeg.setPosition('elbow', self.frontLeftLeg.max_down)
	    elif turningRightState == 9
	    	self.neutral_position()



def wait(seconds):
    initial = rospy.Time.now()
    while rospy.Time.now() < initial + rospy.Duration(seconds):
        continue

# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")

    # control loop running at 10hz
    r = rospy.Rate(10) # 10hz
    for x in range(1, 9):
       setMotorTargetSpeed(x, 500)
        
    Ross = Robot()
    # Ross.walking_position_left()
    turningRightState = 0

    while not rospy.is_shutdown():
        # call function to get sensor value
        port = 5
        sensor_reading = getSensorValue(port)
        # rospy.loginfo("Sensor value at port %d: %f", 5, sensor_reading)

        # call function to set motor position
        motor_id = 1
        target_val = 450
        
        rospy.loginfo(getSensorValue(Ross.head_port))
        
        # Ross.walk()
        turningRightState = Ross.turnRight2(turningRightState)


        
        # response = setMotorTargetPositionCommand(motor_id, target_val)

        # sleep to enforce loop rate
        r.sleep()
