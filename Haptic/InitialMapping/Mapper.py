#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Initialze all servos      *********
#
#
# Module to initialize all servos [AX-12A]
# Things to note: 
# Have servo ID already initialized.
# Protocol 1.0
#   For AX-12A:
#       ADDR_MX_TORQUE_ENABLE      = 24
#       ADDR_MX_GOAL_POSITION      = 30
#       ADDR_MX_PRESENT_POSITION   = 36
# 

import os
import socket

global L 
L = False
global R
R = False
global T1Flag
global T1Change
T1Change = False
T1Flag = True
global T2Flag
global T2Change
T2Change = False
T2Flag = True


def mapperF(data):
    global L
    global R
    thresholdZ = -0.05
    
    if (data[5] >= thresholdZ):
        if(not R):
            print("Left Active")
            z = genMapper([-0.050, 0.100], [320, 75], data[5])
            x = genMapper([-0.080, 0.080], [624, 400], data[3])
            updatePos([4, 10, 15], [inverse(z), inverse(z), z])
            updatePos([1,  7, 14], [inverse(x), inverse(x), x])
            updatePos([2,  8, 13], [inverse(x), inverse(x), x])
            L = True
        else:
            L = False
            print("Left up, but can't move")
    else:
        L = False
        
    if (data[2] >= thresholdZ):
        if(not L):
            print("Right Active")
            z = genMapper([-0.050, 0.100], [320, 75], data[2])
            x = genMapper([-0.080, 0.080], [624, 400], data[0])
            updatePos([3, 9, 16], [z, z, inverse(z)])
            updatePos([1, 7, 14], [x, x, inverse(x)])
            updatePos([2, 8, 13], [x, x, inverse(x)])
            R = True
        else:
            R = False
            print("Right up, but can't move")
    else:
        R = False
    if (not L):
        updatePos([4, 10, 15], [inverse(320), inverse(320), 320])
    elif (not R):
        updatePos([3,  9, 16], [320, 320, inverse(320)])


def mapperT1(data):
    global T1Flag
    global T1Change
    thresholdY = 0.02

    if(data[1] >= thresholdY and T1Flag):
        T1Change = True
        y = genMapper([0.020, 0.080], [320, 75], data[1])
        x = genMapper([-0.080, 0.080], [624, 400], data[0])
        updatePos([4, 10, 15], [inverse(y), inverse(y), y])
        updatePos([1, 7, 14], [inverse(x), inverse(x), inverse(x)])
        updatePos([2, 8, 13], [x, x, x])
    elif(data[1] >= thresholdY and not(T1Flag)):
        T1Change = True
        y = genMapper([0.020, 0.080], [320, 75], data[1])
        x = genMapper([-0.080, 0.080], [624, 400], data[0])
        updatePos([3, 9, 16], [y, y, inverse(y)])
        updatePos([1, 7, 14], [x, x, x])
        updatePos([2, 8, 13], [inverse(x), inverse(x), inverse(x)])
    else:
        if(T1Change):
            T1Flag = not(T1Flag)
            T1Change = False

def mapperT2(data):
    global T2Flag
    global T2Change
    thresholdY = -0.02
    print("Turn2", T2Flag)
    if(data[1] <= thresholdY and T2Flag):
        T2Change = True
        y = genMapper([-0.020, -0.080], [320, 75], data[1])
        x = genMapper([-0.080, 0.080], [624, 400], data[0])
        updatePos([4, 10, 15], [inverse(y), inverse(y), y])
        updatePos([1, 7, 14], [x, x, x])
        updatePos([2, 8, 13], [inverse(x), inverse(x), inverse(x)])
    elif(data[1] <= thresholdY and not(T2Flag)):
        T2Change = True
        y = genMapper([-0.020, -0.080], [320, 75], data[1])
        x = genMapper([-0.080, 0.080], [624, 400], data[0])
        updatePos([3, 9, 16], [y, y, inverse(y)])
        updatePos([1, 7, 14], [inverse(x), inverse(x), inverse(x)])
        updatePos([2, 8, 13], [x, x, x])
    else:
        if(T2Change):
            T2Flag = not(T2Flag)
            T2Change = False

def genMapper(dataFrom, dataTo, dataMap):
    minimum = min(dataFrom)
    maximum = max(dataFrom)
    if dataMap >= maximum:
        dataMap = maximum
    elif dataMap <= minimum:
        dataMap = minimum
    out = (dataTo[1] - dataTo[0])/(dataFrom[1] - dataFrom[0]) * (dataMap - dataFrom[0]) + (dataTo[0])
    return int(out)

def inverse(x):
    return 1023-x

def initializer():
    updatePos([1, 2, 7, 8, 13, 14], [512]*6)
    updatePos([3, 15, 9],[320]*3)
    updatePos([4, 16, 10], [inverse(320)]*3)
    updatePos([11, 17, 5], [850]*3)
    updatePos([12, 18, 6], [inverse(850)]*3)
    Fixers([x+1 for x in range(18)])


def selector(data):
    updatePos([11, 17, 5], [850]*3)
    updatePos([12, 18, 6], [inverse(850)]*3)
    if (data[6]):
        mapperT1(data[0:3])
    elif (data[7]):
        mapperT2(data[3:6])
    else:
        mapperF(data[0:6])

def gatherData():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(('',6666))
    print("UDP waiting")
    
    while True:
        dataFromClient,address = server_socket.recvfrom(256)
        dataFromClient = dataFromClient.split(',')
        #print(dataFromClient)
        try:
            data = [float(x) for x in dataFromClient]
            selector(data)
        except ValueError:
            print ("Value err")
    

# Updates position in servo
def updatePos(ID, pos):
    for  i,p in zip(ID, pos):
        # Allocate goal position value into byte array
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(p)), DXL_HIBYTE(DXL_LOWORD(p))] #Keep a list of 2 (VIMP) 
        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(i, param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % i)
            quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    """
    while 1:
    # Read Dynamixel#1 present position
        presentPos = []
        for i in ID:
            present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, i, ADDR_MX_PRESENT_POSITION)
            presentPos.append(present_position)

            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

        diff = [abs(x - y) for x,y in zip(presentPos , pos)]
        if not (diff > [DXL_MOVING_STATUS_THRESHOLD]*len(pos)):
            break        
    """
# Enables torque to active in servos ID
def Fixers(ID):
    for i in ID:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % i)

# Disables torque in servos ID
def Flexers(ID):
    for i in ID:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))


if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

# Data Byte Length
LEN_MX_GOAL_POSITION = 2
LEN_MX_PRESENT_POSITION = 2

PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default settingTerminator(DXL_ID)
# Terminator(FIXED_DXL_ID)
FIXED_DXL_ID                = [5, 6, 11, 12, 17,18]
DXL_ID                      = [1, 2, 3, 4, 7, 8, 9, 10, 13, 14, 15, 16]

DXL1_ID                     = 17                 # Dynamixel#1 ID : 1
DXL2_ID                     = 7                 # Dynamixel#1 ID : 2
DXL3_ID                     = 11                 # Dynamixel#1 ID : 7
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 300           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 500            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()
'''
Fixers(DXL_ID)
Fixers(FIXED_DXL_ID)

updatePos([7, 9, 11, 8, 10, 12], [500]*6)

while 1:
    print("Yes")

Flexers(DXL_ID)
Flexers(FIXED_DXL_ID)
'''


initializer()
gatherData()

# Close port
portHandler.closePort()
