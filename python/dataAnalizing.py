#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

# Global variables storing the data
messageCount = 0 # Counts the IMU messages because they come in at a lower rate
jointStateErrorSum = np.zeros(4)

# Global variables storing the joint states
exoJointState = JointState()

# Time limit for the data collection
timeLimit = 180 # seconds

# Rate of the imu joint state messages
imuRate = 148 # Hz

# Number of messages to collect
numMessages = timeLimit * imuRate

# Publisher for the joint state error
errorPublisher = rospy.Publisher('/joint_state_error', Float32MultiArray, queue_size=10)

# Error message
errorMessage = Float32MultiArray()

# Mirrored flag
mirrored = True

# EXO Joint State callback
def exoJointStateCallback(msg):
    global messageCount, exoJointState, numMessages

    # Only do error calculations for the first 1000 messages
    if messageCount < numMessages:
        # Store the message into the global variable
        exoJointState = msg

# IMU Joint State callback
def imuJointStateCallback(msg):
    global messageCount, jointStateErrorSum, exoJointState, numMessages, mirrored, errorMessage

    # Only do error calculations for the first 1000 messages
    if messageCount < numMessages:
        # Increment the message count
        messageCount += 1

        # Get the joint state error
        if not mirrored:
            for joint in range(4):
                jointStateErrorSum[joint] += np.abs(msg.position[joint] - exoJointState.position[joint])
        else:
            jointStateErrorSum[0] += np.abs(msg.position[2] - exoJointState.position[0])
            jointStateErrorSum[1] += np.abs(msg.position[3] - exoJointState.position[1])
            jointStateErrorSum[2] += np.abs(msg.position[0] - exoJointState.position[2])
            jointStateErrorSum[3] += np.abs(msg.position[1] - exoJointState.position[3])

        # Set the error message data
        errorMessage.data = jointStateErrorSum

    # Publish the error message after converting to degrees
    # for joint in range(4):
    #     errorMessage.data[joint] = np.degrees(errorMessage.data[joint] / messageCount)

    errorPublisher.publish(errorMessage)

    # Print the average error over the first 1000 messages
    if messageCount == numMessages: # Large range to print the average error
        # Convert the error sum to an average and to degrees
        jointStateErrorSum = jointStateErrorSum / numMessages
        jointStateErrorSum = np.degrees(jointStateErrorSum)

        print("Average error over the first" + numMessages + "messages:")
        print(jointStateErrorSum)
        messageCount += 1

def main():
    # Initialize the node
    rospy.init_node('data_analyzer')

    #?------------------------------ Publishers ------------------------------ #
    rospy.Publisher('/joint_state_error', Float32MultiArray, queue_size=10)

    #? -------------------------------- Subscribers ------------------------------- #
    # IMU data subscriber
    rospy.Subscriber('/imu_joint_states', JointState, imuJointStateCallback)

    # Real exo joint state subscriber
    rospy.Subscriber('/X2_SRA_A/joint_states', JointState, exoJointStateCallback)

    rospy.spin()

if __name__ == '__main__':
    main()
