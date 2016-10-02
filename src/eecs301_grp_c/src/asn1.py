#!/usr/bin/env python
import roslib
import rospy
from fw_wrapper.srv import *

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

	getPosition(self, part):
		if part == 'elbow':
			return getMotorPositionCommand(self.elbow)
		elif part == 'shoulder':
			return getMotorPositionCommand(self.shoulder)
		else:
			rospy.loginfo('Please pass in an elbow or a shoulder')

	setPosition(self, part, target_val, offset = 0):
		if part == 'elbow':
			if self.up == 'plus':
				setMotorTargetPositionCommand(self.elbow, target_val + offset)
			else:
				setMotorTargetPositionCommand(self.elbow, target_val - offset)
		elif part == 'shoulder':
			if self.forward == 'plus':
				setMotorTargetPositionCommand(self.shoulder, target_val + offset)
			else:
				setMotorTargetPositionCommand(self.shoulder, target_val - offset)
		else:
			rospy.loginfo('Please pass in an elbow or a shoulder')

	moveLeg(self):
		shoulder_pos = self.getPosition('shoulder')

		if abs(self.max_back - shoulder_pos) <= 5:
			self.setPosition('elbow', self.max_down, 50)
			self.setPosition('shoulder', self.max_forward)
		elif abs(self.max_forward - shoulder_pos) <= 300:
			self.setPosition('elbow', self.max_down)
			if abs(self.max_forward - shoulder_pos) <= 5:
				self.setPosition('shoulder', self.max_backward)




class Robot:
	def __init__(self):
		self.ir_port = 2,
		self.head_port = 1,
		self.backRightLeg = Leg(8, 4, 'plus', 'plus', 544, 205, 814, 210)
		self.backLeftLeg = Leg(7, 3, 'minus', 'minus', 480, 819, 210, 814)
		self.frontRightLeg = Leg(6, 2, 'minus', 'plus', 819, 480, 210, 814)
		self.frontLeftLeg = Leg(5, 1, 'plus', 'minus', 205, 544, 814, 210)

	def getLegPositions(self):
		leg_positions = {}
	    leg_positions['back_right_shoulder'] = getMotorPositionCommand(self.backRightLeg.shoulder)
	    leg_positions['back_right_elbow'] = getMotorPositionCommand(self.backRightLeg.elbow)
	    leg_positions['back_left_shoulder'] = getMotorPositionCommand(self.backLeftLeg.shoulder)
	    leg_positions['back_right_elbow'] = getMotorPositionCommand(self.backLeftLeg.elbow)
	    leg_positions['front_right_shoulder'] = getMotorPositionCommand(self.frontRightLeg.shoulder)
	    leg_positions['front_right_elbow'] = getMotorPositionCommand(self.frontRightLeg.elbow)
	    leg_positions['front_left_shoulder'] = getMotorPositionCommand(self.frontLeftLeg.shoulder)
	    leg_positions['front_left_elbow'] = getMotorPositionCommand(self.frontLeftLeg.elbow)
	    return leg_positions

	def walking_position(self):
		self.backRightLeg.setPosition('elbow', self.backRightLeg.max_down)
		self.backRightLeg.setPosition('shoulder', self.backRightLeg.max_forward)
		self.backLeftLeg.setPosition('elbow', self.backLeftLeg.max_down)
		self.backLeftLeg.setPosition('shoulder', self.backLeftLeg.max_forward)
		self.frontRightLeg.setPosition('elbow', self.frontRightLeg.max_down)
		self.frontRightLeg.setPosition('shoulder', self.frontRightLeg.max_back)
		self.frontLeftLeg.setPosition('elbow', self.frontLeftLeg.max_down)
		self.frontLeftLeg.setPosition('shoulder', self.frontLeftLeg.max_forward)

	def walk(self):
		self.backRightLeg.moveLeg()
		self.backLeftLeg.moveLeg()
		self.frontRightLeg.moveLeg()
		self.frontLeftLeg.moveLeg()


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

def globalState():
    machine_state = {}
    machine_state['back_right_shoulder'] = getMotorPositionCommand(PORT_MAP['back_right_shoulder'])
    machine_state['back_right_elbow'] = getMotorPositionCommand(PORT_MAP['back_right_elbow'])
    machine_state['back_left_shoulder'] = getMotorPositionCommand(PORT_MAP['back_left_shoulder'])
    machine_state['back_right_elbow'] = getMotorPositionCommand(PORT_MAP['back_right_elbow'])
    machine_state['front_right_shoulder'] = getMotorPositionCommand(PORT_MAP['front_right_shoulder'])
    machine_state['front_right_elbow'] = getMotorPositionCommand(PORT_MAP['front_right_elbow'])
    machine_state['front_left_shoulder'] = getMotorPositionCommand(PORT_MAP['front_left_shoulder'])
    machine_state['front_right_elbow'] = getMotorPositionCommand(PORT_MAP['front_right_elbow'])
    
    return machine_state
    
def setRandomPos():
    setMotorTargetPositionCommand(PORT_MAP['back_right_elbow'], 512)
    setMotorTargetPositionCommand(PORT_MAP['back_right_shoulder'], 512)
    setMotorTargetPositionCommand(PORT_MAP['front_right_elbow'], 512)
    setMotorTargetPositionCommand(PORT_MAP['front_right_shoulder'], 512)
    
def walkingPosRight():
    FORWARD_HIND_POS = MAX_SHOULDER_POS - INNER_OFFSET
    BACKWARD_HIND_POS = MIN_SHOULDER_POS - OUTER_OFFSET
    UP_HIND_ELBOW_POS = MIN_ELBOW_POS + 50
    DOWN_HIND_ELBOW_POS = MIN_ELBOW_POS
    FORWARD_FRONT_POS = MAX_SHOULDER_POS + OUTER_OFFSET
    BACKWARD_FRONT_POS = MIN_SHOULDER_POS + INNER_OFFSET
    UP_FRONT_ELBOW_POS = MAX_ELBOW_POS - 50
    DOWN_FRONT_ELBOW_POS = MAX_ELBOW_POS
    
    setMotorTargetPositionCommand(PORT_MAP['back_right_elbow'], UP_HIND_ELBOW_POS)
    setMotorTargetPositionCommand(PORT_MAP['back_right_shoulder'], FORWARD_HIND_POS)
    setMotorTargetPositionCommand(PORT_MAP['front_right_elbow'], DOWN_FRONT_ELBOW_POS)
    setMotorTargetPositionCommand(PORT_MAP['front_right_shoulder'], BACKWARD_FRONT_POS)
    
def walkingPosLeft():
    FORWARD_HIND_POS = MIN_SHOULDER_POS + INNER_OFFSET
    BACKWARD_HIND_POS = MAX_SHOULDER_POS + OUTER_OFFSET
    UP_HIND_ELBOW_POS = MAX_ELBOW_POS - 50
    DOWN_HIND_ELBOW_POS = MAX_ELBOW_POS
    FORWARD_FRONT_POS = MIN_SHOULDER_POS - OUTER_OFFSET
    BACKWARD_FRONT_POS = MAX_SHOULDER_POS - INNER_OFFSET
    UP_FRONT_ELBOW_POS = MIN_ELBOW_POS + 50
    DOWN_FRONT_ELBOW_POS = MIN_ELBOW_POS
    
    setMotorTargetPositionCommand(PORT_MAP['back_left_elbow'], UP_HIND_ELBOW_POS)
    setMotorTargetPositionCommand(PORT_MAP['back_left_shoulder'], FORWARD_HIND_POS)
    setMotorTargetPositionCommand(PORT_MAP['front_left_elbow'], DOWN_FRONT_ELBOW_POS)
    setMotorTargetPositionCommand(PORT_MAP['front_left_shoulder'], BACKWARD_FRONT_POS)
  
def moveHindLeg(side):
    machine_state = globalState()
    FORWARD_HIND_POS = None
    BACKWARD_HIND_POS = None
    UP_ELBOW_POS = None
    DOWN_ELBOW_POS = None
    SHOULDER_PORT = None
    ELBOW_PORT = None
    SHOULDER_PROP = None
    
    if side == 'right':
        FORWARD_HIND_POS = MAX_SHOULDER_POS - INNER_OFFSET
        BACKWARD_HIND_POS = MIN_SHOULDER_POS - OUTER_OFFSET
        UP_ELBOW_POS = MIN_ELBOW_POS + 50
        DOWN_ELBOW_POS = MIN_ELBOW_POS
        SHOULDER_PROP = 'back_right_shoulder'
        ELBOW_PROP = 'back_right_elbow'
    else:
        FORWARD_HIND_POS = MIN_SHOULDER_POS + INNER_OFFSET
        BACKWARD_HIND_POS = MAX_SHOULDER_POS + OUTER_OFFSET
        UP_ELBOW_POS = MAX_ELBOW_POS - 50
        DOWN_ELBOW_POS = MAX_ELBOW_POS
        SHOULDER_PROP = 'back_left_shoulder'
        ELBOW_PROP = 'back_left_elbow'
        
    if abs(FORWARD_HIND_POS - machine_state[SHOULDER_PROP]) <= 300:
        setMotorTargetPositionCommand(PORT_MAP[ELBOW_PROP], DOWN_ELBOW_POS)
        
    if abs(BACKWARD_HIND_POS - machine_state[SHOULDER_PROP]) <= 5:
        setMotorTargetPositionCommand(PORT_MAP[ELBOW_PROP], UP_ELBOW_POS)
        setMotorTargetPositionCommand(PORT_MAP[SHOULDER_PROP], FORWARD_HIND_POS)
    elif abs(FORWARD_HIND_POS - machine_state[SHOULDER_PROP]) <= 5:
        setMotorTargetPositionCommand(PORT_MAP[SHOULDER_PROP], BACKWARD_HIND_POS)
        
        
    
def moveFrontLeg(side):
    machine_state = globalState()
    FORWARD_FRONT_POS = None
    BACKWARD_FRONT_POS = None
    UP_ELBOW_POS = None
    DOWN_ELBOW_POS = None
    SHOULDER_PORT = None
    ELBOW_PORT = None
    SHOULDER_PROP = None
    
    if side == 'right':
        FORWARD_FRONT_POS = MAX_SHOULDER_POS + OUTER_OFFSET
        BACKWARD_FRONT_POS = MIN_SHOULDER_POS + INNER_OFFSET
        UP_ELBOW_POS = MAX_ELBOW_POS - 50
        DOWN_ELBOW_POS = MAX_ELBOW_POS
        SHOULDER_PROP = 'front_right_shoulder'
        ELBOW_PROP = 'front_right_elbow'
    else:
        FORWARD_FRONT_POS = MIN_SHOULDER_POS - OUTER_OFFSET
        BACKWARD_FRONT_POS = MAX_SHOULDER_POS - INNER_OFFSET
        UP_ELBOW_POS = MIN_ELBOW_POS + 50
        DOWN_ELBOW_POS = MIN_ELBOW_POS
        SHOULDER_PROP = 'front_left_shoulder'
        ELBOW_PROP = 'front_left_elbow'
        
    if abs(FORWARD_FRONT_POS - machine_state[SHOULDER_PROP]) <= 300:
        setMotorTargetPositionCommand(PORT_MAP[ELBOW_PROP], DOWN_ELBOW_POS)
        
    if abs(BACKWARD_FRONT_POS - machine_state[SHOULDER_PROP]) <= 5:
        setMotorTargetPositionCommand(PORT_MAP[ELBOW_PROP], UP_ELBOW_POS)
        setMotorTargetPositionCommand(PORT_MAP[SHOULDER_PROP], FORWARD_FRONT_POS)
    elif abs(FORWARD_FRONT_POS - machine_state[SHOULDER_PROP]) <= 5:
        setMotorTargetPositionCommand(PORT_MAP[SHOULDER_PROP], BACKWARD_FRONT_POS)

# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")

    # control loop running at 10hz
    r = rospy.Rate(10) # 10hz
    for x in range(1, 9):
        setMotorTargetSpeed(x, 200)
    walkingPosRight()
    walkingPosLeft()
    while not rospy.is_shutdown():
        # call function to get sensor value
        port = 5
        sensor_reading = getSensorValue(port)
        # rospy.loginfo("Sensor value at port %d: %f", 5, sensor_reading)

        # call function to set motor position
        motor_id = 1
        target_val = 450
        moveHindLeg('right')
        moveHindLeg('left')
        moveFrontLeg('right')
        moveFrontLeg('left')
        # response = setMotorTargetPositionCommand(motor_id, target_val)

        # sleep to enforce loop rate
        r.sleep()
