#!/usr/bin/env python
import roslib
import signal
import rospy
import sys
import os
import time
from fw_wrapper.srv import *
import operator
from map import *
from path import *
from mltest import *
from itertools import groupby
import numpy as np
import csv


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
HEAD_PORT = 4
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

def wait(seconds):
    initial = rospy.Time.now()
    while rospy.Time.now() < initial + rospy.Duration(seconds):
        continue
        
def timeout(iterations, func, *args):
    while iterations > 0:
        func(*args)
        iterations -= 1

def shutdown(sig, stackframe):
    rospy.loginfo("Setting wheels to zero")
    for wheel in [13]:
        setMotorWheelSpeed(wheel, 0)
    sys.exit(0)
    
def appendSensorValue(port, arr):
    arr.append(getSensorValue(port))
    
def clean_data(filename):
    with open(filename, 'rb') as f:
        reader = csv.reader(f)
        for row in reader:
            print row

# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")

    signal.signal(signal.SIGINT, shutdown)

    # control loop running at 10hz
    r = rospy.Rate(10) # 10hz
   
    setMotorMode(13, 1)
    
    args = sys.argv[1:]
    print args
    try:
        mode = args[0]
        
        if mode == 'train':
            filename = args[1]
            name = args[2]
            csv_file = open(filename, 'a')
            wr = csv.writer(csv_file)
        
            for x in range(0, 50):
                readings = []
                setMotorWheelSpeed(13, 120)
                timeout(320, appendSensorValue, HEAD_PORT, readings)
                setMotorWheelSpeed(13, 0)
                wr.writerow([name] + readings)
                print "Reading %d: Wrote %d columns to %s" % (x + 1, len(readings), filename)
        elif mode == 'identify':
            name = args[1]
            timestamp = str(time.time())
            writefile = 'tests/test-' + timestamp + '.csv'
            csv_file = open(writefile, 'a')
            wr = csv.writer(csv_file)
            
            for x in range(0, 5):
                readings = []
                setMotorWheelSpeed(13, 120)
                timeout(320, appendSensorValue, HEAD_PORT, readings)
                setMotorWheelSpeed(13, 0)
                wr.writerow([name] + readings)
                print "Reading %d: Wrote %d columns to %s" % (x + 1, len(readings), writefile)
            
            csv_file.close()
            training_dataset = loadDataset(os.path.join(os.path.dirname(__file__), "../../..", "training_data.csv"))
            cleanDataAvg(training_dataset,os.path.join(os.path.dirname(__file__), "../../..", "clean_training_data.csv"))
            discrete_training = discretizeData(training_dataset, os.path.join(os.path.dirname(__file__), "../../..", "discrete_training_data.csv"))

            test_dataset = loadDataset(os.path.join(os.path.dirname(__file__), "../../..", writefile))
            cleanDataAvg(test_dataset,os.path.join(os.path.dirname(__file__), "../../..", "clean_test_data.csv"))
            discrete_test = discretizeData(test_dataset, os.path.join(os.path.dirname(__file__), "../../..", "discrete_test_data.csv"))

            cube_dataset = loadDataset(os.path.join(os.path.dirname(__file__), "../../..", 'data/Rubiks.csv'))
            triangle_dataset = loadDataset(os.path.join(os.path.dirname(__file__), "../../..", 'data/Toblerone.csv'))
            cylinder_dataset = loadDataset(os.path.join(os.path.dirname(__file__), "../../..", 'data/Starbucks.csv'))
            
            #evaluateClassification(training_dataset, test_dataset, 50, "Cylinder")
            
            classifyObjects(discrete_training, discrete_test)
    except IndexError:
        print "Please enter valid arguments"
    
   # while not rospy.is_shutdown():
   
        
