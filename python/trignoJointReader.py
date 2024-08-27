#!/usr/bin/env python3
from typing import List
import numpy as np
import rospy
from trigno_msgs.msg import trignoMultiIMU, trignoIMU
from sensor_msgs.msg import JointState
from math import degrees, radians
from collections import deque
from rospkg import RosPack
import yaml

# List of IMUs and EMGs and their associated data
numSensors = 8
imuLocations = np.zeros(numSensors)

averagedIMUData = [0, 0, 0, 0]

#* -------------------------------- Publishers -------------------------------- #
# Joint state publisher
jointStatePublisher = rospy.Publisher('/imu_joint_states', JointState, queue_size=10)

# Publisher to exo joint states to test code
# jointStatePublisher = rospy.Publisher('/X2_SRA_A/joint_states', JointState, queue_size=10)

#* ------------------------------ Global messages ----------------------------- #
# Joint state message
imujointStateMessage = JointState()
imujointStateMessage.name = ["left_hip_joint", "left_knee_joint",
                            "right_hip_joint", "right_knee_joint",
                            "world_to_backpack"]
imujointStateMessage.position = [0, 0, 0, 0, 0]
imujointStateMessage.velocity = [0, 0, 0, 0, 0]
imujointStateMessage.effort = [0, 0, 0, 0, 0]

# List of current exo joint data
exoJointDataCUR = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

#* -------------------- Global constants from the yaml file ------------------- #
x2ParamsFileName = RosPack().get_path('CORC') + '/config/x2_params.yaml'
with open(x2ParamsFileName, 'r') as file:
    x2Params = yaml.safe_load(file)

# Getting the joint limits
jointLimits = [x2Params["X2_SRA_A"]['joint_position_limits']['hip_min'],
                x2Params["X2_SRA_A"]['joint_position_limits']['hip_max'],
                x2Params["X2_SRA_A"]['joint_position_limits']['knee_min'],
                x2Params["X2_SRA_A"]['joint_position_limits']['knee_max']]

#* ----------------------- List of data for exo and imus ---------------------- #
# Order of the data lists
# # left thigh, left calf, right thigh, right calf, back
exoJointData = [deque(maxlen=100) for i in range(5)] # From the exo
jointStateData = [deque(maxlen=100) for i in range(5)] # From IMUs
signedError = [deque(maxlen=100) for i in range(5)] # Error between the two

#* ----------------------------- Helper functions ----------------------------- #
# Creating a rotation matrix from a quaternion
def rotMatrixFromQuat(q):
    rotMatrix = np.zeros((3,3))
    rotMatrix[0,0] = 1 - 2*q.y*q.y - 2*q.z*q.z
    rotMatrix[0,1] = 2*q.x*q.y - 2*q.z*q.w
    rotMatrix[0,2] = 2*q.x*q.z + 2*q.y*q.w
    rotMatrix[1,0] = 2*q.x*q.y + 2*q.z*q.w
    rotMatrix[1,1] = 1 - 2*q.x*q.x - 2*q.z*q.z
    rotMatrix[1,2] = 2*q.y*q.z - 2*q.x*q.w
    rotMatrix[2,0] = 2*q.x*q.z - 2*q.y*q.w
    rotMatrix[2,1] = 2*q.y*q.z + 2*q.x*q.w
    rotMatrix[2,2] = 1 - 2*q.x*q.x - 2*q.y*q.y

    return rotMatrix

# Get sagittal angle from the IMU
RA0list = [None, None, None, None, None, None, None, None]
RABlist = [None, None, None, None, None, None, None, None]
def getSagittalAngle(imuData: List[trignoIMU], curReading: int):
    # Getting global variables
    global RA0list, RABlist, numSensors

    # Checking if the quaternion is all 0s, if so this is a test message
    if imuData.q[-1].x == 0 and imuData.q[-1].y == 0 and imuData.q[-1].z == 0 and imuData.q[-1].w == 0:
        return 0

    # Only using the most recent data from the IMU
    orientation = imuData.q[curReading]

    # Normalizing the quaternion
    norm = np.linalg.norm([orientation.x, orientation.y, orientation.z, orientation.w])
    orientation.x = orientation.x / norm
    orientation.y = orientation.y / norm
    orientation.z = orientation.z / norm
    orientation.w = orientation.w / norm

    # Transforming the quaternion to a rotation matrix
    R0B = rotMatrixFromQuat(orientation)

    # Only calculating the initial rotation matrix once
    if RA0list[imuData.imu_id - 1] is None:
        # Getting R0A
        RA0list[imuData.imu_id - 1] = rotMatrixFromQuat(imuData.q0).T

    # Getting RAB
    RAB = RA0list[imuData.imu_id - 1] * R0B

    # Getting the rotation from the thigh to the calf based on the number of IMUs
    if numSensors == 8:
        # Getting the sagittal angle from the rotation matrix
        sagittalAngle = np.arcsin(RAB[2, 2])

        # Rotating the transforms of the sensors on the back of the leg
        if imuData.imu_id % 2 == 0:
            sagittalAngle = np.arcsin(-RAB[2, 2])

    # Store the rotation matrix in the list for future use
    RABlist[imuData.imu_id - 1] = 1

    return sagittalAngle

# Adding in failsafes to the joint state messages
def failsafes(tempJointAngles: List[float]):
    global imujointStateMessage, numSensors, jointLimits
    # Setting joint limits
    if numSensors == 8 or numSensors == 10:
        tempJointAngles[0] = max(tempJointAngles[0], radians(jointLimits[0]))
        tempJointAngles[0] = min(tempJointAngles[0], radians(jointLimits[1]))
        tempJointAngles[1] = max(tempJointAngles[1], radians(jointLimits[2]))
        tempJointAngles[1] = min(tempJointAngles[1], radians(jointLimits[3]))
        tempJointAngles[2] = max(tempJointAngles[2], radians(jointLimits[0]))
        tempJointAngles[2] = min(tempJointAngles[2], radians(jointLimits[1]))
        tempJointAngles[3] = max(tempJointAngles[3], radians(jointLimits[2]))
        tempJointAngles[3] = min(tempJointAngles[3], radians(jointLimits[3]))

    elif numSensors == 5:
        for i in range(4):
            if i % 2 == 0:
                imujointStateMessage.position[i] = max(imujointStateMessage.position[i], radians(jointLimits[0]))
                imujointStateMessage.position[i] = min(imujointStateMessage.position[i], radians(jointLimits[1]))
            else:
                imujointStateMessage.position[i] = max(imujointStateMessage.position[i], radians(jointLimits[2]))
                imujointStateMessage.position[i] = min(imujointStateMessage.position[i], radians(jointLimits[3]))

#* --------------------------------- Callbacks -------------------------------- #
# IMU callback function
def imu_callback(IMUDataList: trignoMultiIMU):
    #? Getting global variables
    global imuLocations
    global jointStateData, lineListJointStates
    global exoJointData, lineListExo, exoJointDataCUR
    global signedError, lineListError
    global drawCount, timeList
    global figError, figJoint
    global exoDataList, delayList

    #* ----------------------- Updating the joint states ---------------------- #
    # Updating the joint state message time
    imujointStateMessage.header.stamp = rospy.Time.now()

    # Declaring orderedList
    orderedIMUList = []

    # Reordering the IMU data so the order is back, thighs, calves
    if (len(IMUDataList.trigno_imu) == 8 or len(IMUDataList.trigno_imu) == 10):
        orderedIMUList = [IMUDataList.trigno_imu[0], IMUDataList.trigno_imu[1], IMUDataList.trigno_imu[2],
                            IMUDataList.trigno_imu[3], IMUDataList.trigno_imu[4], IMUDataList.trigno_imu[5],
                            IMUDataList.trigno_imu[6], IMUDataList.trigno_imu[7]]

    # Getting the number of readings in this packet
    numReadings = len(orderedIMUList[0].q)

    # Looping through the number of readings
    for curReading in range(numReadings):
        # Extract the data from the multi message backwards so the back sensor is done first
        for data in orderedIMUList:
            # Get the sagittalAngle from the IMU
            imuLocations[data.imu_id - 1] = getSagittalAngle(data, curReading)

        # If there are 8 IMUs, then average the angles on the same section of the leg
        if (len(IMUDataList.trigno_imu) == 8 or len(IMUDataList.trigno_imu) == 10):
            # Average the sensors angles on the same section of the leg
            averagedIMUData[0] = (imuLocations[0] + imuLocations[1]) / 2
            averagedIMUData[2] = -(imuLocations[3] + imuLocations[4]) / 2
            # Only done for the thighs because calves only have 1 sensor on them
            averagedIMUData[1] = -imuLocations[2]
            averagedIMUData[3] = imuLocations[5]

        # From sagittal angle, derive joint angles and storing them in a temp list
        tempJointAngles = [0, 0, 0, 0, 0]

        # Set the imu backpack joint angle to = the exo backpack joint angle for testing
        tempJointAngles[4] = 0

        # Hip joints are IMU angles - backpack angle
        tempJointAngles[0] = averagedIMUData[0] - tempJointAngles[4]
        tempJointAngles[2] = averagedIMUData[2] - tempJointAngles[4]

        # Knee joints are IMU angles - hip angles
        tempJointAngles[1] = -1 * (averagedIMUData[1] + imujointStateMessage.position[0])
        tempJointAngles[3] = -1 * (averagedIMUData[3] + imujointStateMessage.position[2])

        # Implimenting failsafes for the joint angles
        failsafes(tempJointAngles)

        # Calulating the joint velocities
        for i in range(5):
            imujointStateMessage.velocity[i] = tempJointAngles[i] - imujointStateMessage.position[i]

        # Updating the joint state message
        imujointStateMessage.position = tempJointAngles

        # Publish the joint state message
        jointStatePublisher.publish(imujointStateMessage)

        # Perform actions for each of the 5 joints
        for i in range(5):
            # Append the data to the data arrays
            jointStateData[i].append(degrees(imujointStateMessage.position[i]))

            # Getting the current exo data
            exoJointData[i].append(degrees(exoJointDataCUR[i]))

            # Calculate the signed error
            signedError[i].append(jointStateData[i][-1] - exoJointData[i][-1])

# Creating a list of data points to sync up the IMU and exo data
numDelayedValues = 70
exoDataList = deque(maxlen=numDelayedValues)
def realExoCallback(jointState: JointState):
    #? Getting global variables
    global exoJointDataCUR
    global delay, prevTime
    global exoDataList, numDelayedValues

    # If the queue is empty, then append the current data to fill the queue
    if len(exoDataList) == 0:
        for i in range(numDelayedValues):
            exoDataList.append(jointState.position)

    # Update the exo joint state data
    for i in range(5):
        # Updating the current data, then only the most recent data is used in graphing
        # This is to sync up the number of data points from the IMU to the exo
        exoJointDataCUR[i] = jointState.position[i]

    # Append the exo data to the deque
    exoDataList.append(jointState.position)

def main():
    # Initialize the node
    rospy.init_node('trigno_joint_reader')

    #? -------------------------------- Subscribers ------------------------------- #
    # IMU data subscriber
    rospy.Subscriber('/X2_SRA_A/trigno_imu', trignoMultiIMU, imu_callback)

    # Real exo joint state subscriber
    rospy.Subscriber('/joint_states', JointState, realExoCallback)

    rospy.spin()

if __name__ == '__main__':
    main()
