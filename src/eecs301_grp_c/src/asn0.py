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
    
def performStandingPosition():
    NEUTRAL_MIN_ELBOW_POS = MIN_ELBOW_POS + 40
    NEUTRAL_MAX_ELBOW_POS = MAX_ELBOW_POS - 40
    setMotorTargetPositionCommand(5, NEUTRAL_MIN_ELBOW_POS)
    setMotorTargetPositionCommand(8, NEUTRAL_MIN_ELBOW_POS)
    setMotorTargetPositionCommand(6, NEUTRAL_MAX_ELBOW_POS)
    setMotorTargetPositionCommand(7, NEUTRAL_MAX_ELBOW_POS)
    setMotorTargetPositionCommand(4, MIN_SHOULDER_POS)
    setMotorTargetPositionCommand(3, MAX_SHOULDER_POS)
    setMotorTargetPositionCommand(2, MAX_SHOULDER_POS)
    setMotorTargetPositionCommand(1, MIN_SHOULDER_POS)
    
def performLayingPosition():
    difference = 150
    setMotorTargetPositionCommand(5, MIN_ELBOW_POS + difference)
    setMotorTargetPositionCommand(8, MIN_ELBOW_POS + difference)
    setMotorTargetPositionCommand(6, MAX_ELBOW_POS - difference)
    setMotorTargetPositionCommand(7, MAX_ELBOW_POS - difference)

def performSit(sitting_state):
    if getSensorValue(IR_PORT) > 400:
        performLayingPosition()
        return True
    elif sitting_state:
        performStandingPosition()
        return False

def performWave(sitting_state):
    rospy.loginfo(getMotorPositionCommand(6))
    if not sitting_state and getSensorValue(HEAD_PORT) > 2000:
        difference = 40
        setMotorTargetPositionCommand(7, MAX_ELBOW_POS - difference)
        setMotorTargetPositionCommand(5, MIN_ELBOW_POS + difference)
        setMotorTargetPositionCommand(8, MIN_ELBOW_POS + difference)
        setMotorTargetPositionCommand(6, MIN_ELBOW_POS)
        wait(1.5)
        setMotorTargetPositionCommand(2, MAX_SHOULDER_POS - 100)
        wait(0.1)
        setMotorTargetPositionCommand(2, MAX_SHOULDER_POS)
        wait(0.1)
        setMotorTargetPositionCommand(2, MAX_SHOULDER_POS - 100)
        wait(0.1)
        setMotorTargetPositionCommand(2, MAX_SHOULDER_POS)
        wait(0.1)
        setMotorTargetPositionCommand(6, MAX_ELBOW_POS)
        wait(1.5)

def wait(seconds):
    initial = rospy.Time.now();
    while rospy.Time.now() < initial + rospy.Duration(seconds):
        continue
        
        

# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")

    # control loop running at 10hz
    r = rospy.Rate(10) # 10hz
    sitting_state = False
    waving_state = False

    for x in range(1, 9):
        setMotorTargetSpeed(x, 200)
    while not rospy.is_shutdown():
        # call function to get sensor value
        port = 1
        sensor_reading = getSensorValue(port)
        rospy.loginfo("Sensor value at port %d: %f", port, sensor_reading)

        # call function to set motor position
        # motor_id = 3
        # target_val = 744
        
        
        sitting_state = performSit(sitting_state)
        performWave(sitting_state)
        
        # performStandingPosition()
        # performLayingPosition()
        # rospy.loginfo(getMotorWheelSpeed(motor_id))

        # sleep to enforce loop rate
        r.sleep()
