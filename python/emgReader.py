#!/usr/bin/env python3
import rospy
import numpy as np
import dynamic_reconfigure.server
from collections import deque
from yaml import safe_load
from rospkg import RosPack
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from trigno_msgs.msg import trignoMultiEMG, trignoEMG
from std_srvs.srv import Empty, EmptyResponse
from imu_exo_control.cfg import EMGReaderConfig

#*--------------------- Dynamic reconfigurable variables --------------------- #
# Length of time in seconds to calibrate the EMG
calibrationPeriodDyn = 30

# Number of readings to use for the moving window
windowSizeDyn = 500

# The number of readings to use for the averaging window for muscle activation
muscleActivationWindowSizeDyn = 30

# Variable to change the method used to calculate stiffness
stiffnessMethodDyn = 0

# The current muscle to calculate the stiffness for
curMuscleDyn = 0

# Mapping of cur muscle to EMG id
muscleToEMG = {0: 0, 1: 1, 2: 6, 3: 3, 4: 4, 5: 7}

#* ----------------------------- Global Variables ----------------------------- #
# Current joint state data from the IMUs
jointStateData = JointState()

# A toggle to go from publishing muscle activation to recording the max EMG reading
calibratingEMG = False

# Flag if calibration has been done
calibrationDone = True

# A starting time for the calibration
startTime = 0.0

# List of EMG readings
# Order: Left Thigh Front, Left Thigh Back, Left Calf Front, Left Calf Back,
#        Right Thigh Front, Right Thigh Back, Right Calf Front, Right Calf Back
emgReadingsList = [deque(maxlen=windowSizeDyn) for _ in range(8)]

# List of calculated muscle activations
muscleActivationList = [0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0]

# History of muscle activations for setting the min and max muscle activation
muscleActivationHistory = [deque(maxlen=muscleActivationWindowSizeDyn) for _ in range(8)]

# Last valid stiffness array
lastValidStiffnessArray = np.array([0.0, 0.0, 0.0, 0.0])

# Getting an instance of the rospack
rospack = RosPack()

# Get the location of the package
packagePath = rospack.get_path('imu_exo_control')

# Get the location of the EMG calibration yaml file
muscleActivationCalibrationFile = packagePath + '/config/muscleActivationConfig.yaml'

# Have default values for the max and min muscle activation
maxMuscleActivation = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
minMuscleActivation = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]

# Read the max and min muscle activation from the yaml file
with open(muscleActivationCalibrationFile, 'r') as file:
    # Load the yaml file
    calibrationData = safe_load(file)

    # Get the max and min muscle activation
    maxMuscleActivation = np.array(calibrationData['maxMuscleActivation'])
    minMuscleActivation = np.array(calibrationData['minMuscleActivation'])

#? -------------------------------- Publishers and messages -------------------------------- #
# Publisher for the % of muscle activation
muscleActivationPublisher = rospy.Publisher('muscle_activation', Float32MultiArray, queue_size=10)

# Muscle activation message
muscleActivationMessage = Float32MultiArray()
muscleActivationMessage.data = [0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0]

# Unscaled muscle activation message
unscaledMuscleActivationPublisher = rospy.Publisher('unscaled_muscle_activation', Float32MultiArray, queue_size=10)

# Unscaled muscle activation message
unscaledMuscleActivationMessage = Float32MultiArray()
unscaledMuscleActivationMessage.data = [0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0]

# Publisher for the stiffness message
stiffnessPublisher = rospy.Publisher('stiffness', Float32MultiArray, queue_size=10)

# Stiffness message
stiffnessArrayMessage = Float32MultiArray()
stiffnessArrayMessage.data = [0.0, 0.0, 0.0, 0.0]

#* ----------------------- Stiffness calculation methods ---------------------- #
# Generate the pairings of muscle readings of antagonistic muscles
def generateAntagonisticMusclePairs():
    # Generate the pairings of muscle readings of antagonistic muscles
    # Left hip and left thigh back
    # Left knee and left calf back
    # Right knee and right calf back
    # Right hip and right thigh back
    return [np.array([muscleActivationList[0], muscleActivationList[6]]),
            np.array([muscleActivationList[0], muscleActivationList[1]]),
            np.array([muscleActivationList[3], muscleActivationList[7]]),
            np.array([muscleActivationList[3], muscleActivationList[4]])]

# Max contraction method
def maxMuscleContration():
    global muscleActivationMessage, stiffnessArrayMessage

    # Generate the pairings of muscle readings of antagonistic muscles
    musclePairReadings = generateAntagonisticMusclePairs()

    # Calculate the stiffness
    for i in range(4):
        # Add the muscle pair readings
        stiffnessArrayMessage.data[i] = max(musclePairReadings[i])

# Min contraction method
def minMuscleContration():
    global muscleActivationMessage, stiffnessArrayMessage

    # Generate the pairings of muscle readings of antagonistic muscles
    musclePairReadings = generateAntagonisticMusclePairs()

    # Calculate the stiffness
    for i in range(4):
        # Add the muscle pair readings
        stiffnessArrayMessage.data[i] = min(musclePairReadings[i])

# High low method
def highLowMethod():
    global muscleActivation, stiffnessArrayMessage

    # Generate the pairings of muscle readings of antagonistic muscles
    musclePairReadings = generateAntagonisticMusclePairs()

    # Calculate the stiffness
    for i in range(4):
        stiffnessArrayMessage.data[i] = min(musclePairReadings[i]) / max(musclePairReadings[i]) \
                                                                        + sum(musclePairReadings[i])

# Contributing muscle method
def contributingMuscleMethod():
    global muscleActivationList, jointStateData, stiffnessArrayMessage

    # Generate the pairings of muscle readings of antagonistic muscles
    musclePairReadings = generateAntagonisticMusclePairs()

    # Calculating the stiffness based on the contributing muscle method
    # Example:
    # If the left hip is moving forward, the left thigh front is contributing
    # If the left hip is moving back, the left thigh back is contributing

    # Checking each joint velocity and setting the stiffness accordingly
    for i in range(4):
        if jointStateData.velocity[i] > 0:
            # Left hip moving forward
            stiffnessArrayMessage.data[i] = musclePairReadings[i][0]
        else:
            # Left hip moving back
            stiffnessArrayMessage.data[i] = musclePairReadings[i][1]

# Just the front of the thigh to control the knee stiffness
def justFrontThigh():
    global muscleActivationList, stiffnessArrayMessage

    # Calculate the stiffness
    stiffnessArrayMessage.data[1] = muscleActivationList[0]
    stiffnessArrayMessage.data[3] = muscleActivationList[3]

    # Set the other stiffness to 50%
    stiffnessArrayMessage.data[0] = 50.0
    stiffnessArrayMessage.data[2] = 50.0

# Calculate the muscle activation based on the last 10 readings
def calculateMuscleActivation(curEMG: trignoEMG):
    global maxMuscleActivation, minMuscleActivation, emgReadingsList, zeroCount, calibrationDone, windowSizeDyn

    # Get the last few readings for this emg
    recentEMGReadings = emgReadingsList[curEMG.emg_id - 1]

    # Calculate the rms of the last few readings
    reading = np.sqrt(np.mean(np.square(recentEMGReadings)))

    # Add the muscle activation to the history
    muscleActivationHistory[curEMG.emg_id - 1].append(reading)

    # If the calibration toggle is on, update the max EMG reading
    if calibratingEMG:
        # Update the max EMG reading if a higher one has been reached
        maxMuscleActivation[curEMG.emg_id - 1] = max(maxMuscleActivation[curEMG.emg_id - 1], reading)

        # Update the min EMG reading if a lower one has been reached
        minMuscleActivation[curEMG.emg_id - 1] = min(minMuscleActivation[curEMG.emg_id - 1], reading)

    # If the calibration is done, scale the reading
    if calibrationDone:
        # # If the max and min muscle activation are the same, add a small value to the max muscle activation
        # if maxMuscleActivation[curEMG.emg_id - 1] == minMuscleActivation[curEMG.emg_id - 1]:
        #     finalReading = 100 * (reading - minMuscleActivation[curEMG.emg_id - 1])\
        #                         / 1
        # else:
        # Store final reading
        finalReading =  100 * (reading - minMuscleActivation[curEMG.emg_id - 1])\
                                / (maxMuscleActivation[curEMG.emg_id - 1] - minMuscleActivation[curEMG.emg_id - 1])
    else:
        finalReading = 1e-5 # Set to a very small value to avoid division by zero

    # Ensure that the reading is not negative
    reading = max(reading, 0.0)

    # Store the scaled reading
    muscleActivationList[curEMG.emg_id - 1] = finalReading

    # return finalReading, recentEMGReadings
    return finalReading, reading

#* --------------------------- Subscriber callbacks --------------------------- #
# EMG callback function
def emg_callback(EMGDataList):
    # Getting global variables
    global timeList, axBack, axLeftThigh, axLeftCalf, axRightThigh, axRightCalf
    global backData, leftThighFrontData, leftCalfData, rightThighData, rightCalfData
    global drawCount, calibratingEMG, maxMuscleActivation, startTime
    global emgReadingsList, calibrationDone
    global unscaledMuscleActivationMessage, muscleActivationMessage
    global calibrationPeriodDyn

    #? ------------------------------- EMG loop ------------------------------- #
    for curEMG in EMGDataList.trigno_emg:
        # Append the EMG reading to the reading list
        for reading in curEMG.emg:
            emgReadingsList[curEMG.emg_id - 1].append(reading)

        # Adding the muscle activation calculation to the muscle activation message
        scaledMuscleActivation, unscaledMuscleActivation = calculateMuscleActivation(curEMG)

        muscleActivationMessage.data[curEMG.emg_id - 1] = scaledMuscleActivation
        unscaledMuscleActivationMessage.data[curEMG.emg_id - 1] = unscaledMuscleActivation

    # Cuttoff the muscle activation at 0
    muscleActivationMessage.data = np.maximum(muscleActivationMessage.data, 0.0)

    # Calculate the stiffness based on the method
    if stiffnessMethodDyn == 0:
        maxMuscleContration()
    elif stiffnessMethodDyn == 1:
        minMuscleContration()
    elif stiffnessMethodDyn == 2:
        highLowMethod()
    elif stiffnessMethodDyn == 3:
        contributingMuscleMethod()
    elif stiffnessMethodDyn == 4:
        justFrontThigh()

    # Publish the muscle activation
    muscleActivationPublisher.publish(muscleActivationMessage)
    unscaledMuscleActivationPublisher.publish(unscaledMuscleActivationMessage)

    if not calibratingEMG:
        # Setting bounds for stiffness
        for i in range(4):
            # Add bounds to the stiffness
            stiffnessArrayMessage.data[i] = np.min([stiffnessArrayMessage.data[i], 100.0])
            stiffnessArrayMessage.data[i] = np.max([stiffnessArrayMessage.data[i], 0.0])

            # Check if the stiffness is NaN
            if np.isnan(stiffnessArrayMessage.data[i]):
                stiffnessArrayMessage.data[i] = lastValidStiffnessArray[i]
            else:
                # Store this value in last stiffness array
                lastValidStiffnessArray[i] = stiffnessArrayMessage.data[i]

        # Publish the stiffness
        stiffnessPublisher.publish(stiffnessArrayMessage)

    # If the calibration time has passed, stop calibrating the EMG
    if calibratingEMG and rospy.Time.now().to_sec() - startTime > calibrationPeriodDyn:
        # Set the flag to stop calibrating the EMG
        calibratingEMG = False

        # Open the yaml file to store the calibration data
        with open(muscleActivationCalibrationFile, 'w') as file:
            # Write the calibration data to the yaml file
            file.write('maxMuscleActivation: ' + str(maxMuscleActivation.tolist()) + '\n')
            file.write('minMuscleActivation: ' + str(minMuscleActivation.tolist()) + '\n')

        # Reset the start time
        startTime = 0.0

        # Set the calibration done flag
        calibrationDone = True

# IMU joint state callback
def joint_state_callback(jointStateIn: JointState):
    # Store the current joint state
    global jointStateData

    jointStateData = jointStateIn

#* ----------------------------- Service callbacks ---------------------------- #
# Service to set the max muscle activation to the current muscle activation for the selected muscle
def setMaxMuscleActivation(request: Empty):
    global muscleActivationList, maxMuscleActivation, curMuscleDyn, calibrationDone

    # Set the max muscle activation to the current muscle activation
    maxMuscleActivation[curMuscleDyn] = np.average(muscleActivationHistory[curMuscleDyn])

    # Write the max muscle activation to the yaml file
    with open(muscleActivationCalibrationFile, 'w') as file:
        # Write the calibration data to the yaml file
        file.write('maxMuscleActivation: ' + str(maxMuscleActivation.tolist()) + '\n')
        file.write('minMuscleActivation: ' + str(minMuscleActivation.tolist()) + '\n')

    # Set the calibration done flag
    calibrationDone = True

    return EmptyResponse()

# Service to set the min muscle activation to the current muscle activation for the selected muscle
def setMinMuscleActivation(request: Empty):
    global muscleActivationList, minMuscleActivation, curMuscleDyn, calibrationDone

    # Set the min muscle activation to the current muscle activation
    minMuscleActivation[curMuscleDyn] = np.average(muscleActivationHistory[curMuscleDyn])

    # Write the min muscle activation to the yaml file
    with open(muscleActivationCalibrationFile, 'w') as file:
        # Write the calibration data to the yaml file
        file.write('maxMuscleActivation: ' + str(maxMuscleActivation.tolist()) + '\n')
        file.write('minMuscleActivation: ' + str(minMuscleActivation.tolist()) + '\n')

    # Set the calibration done flag
    calibrationDone = True

    return EmptyResponse()

# The calibration service to update the max EMG reading for each muscle
def calibrateEMGCallback(request: Empty):
    global calibratingEMG, maxMuscleActivation, minMuscleActivation, startTime

    # Activate a toggle to go from publishing muscle activation to recording the max EMG reading
    calibratingEMG = True

    # Reset the max and min EMG readings
    maxMuscleActivation = np.array([-1.0, -1.0, -1.0, -1.0,
                                -1.0, -1.0, -1.0, -1.0])
    minMuscleActivation = np.array([1.0, 1.0, 1.0, 1.0,
                                1.0, 1.0, 1.0, 1.0])

    # Start a 10 second timer to calibrate the EMG
    startTime = rospy.Time.now().to_sec()

    return EmptyResponse()

# Read the calibration data from the yaml file
def readCalibrationDataCallaback(request: Empty):
    global maxMuscleActivation, minMuscleActivation

    # Read the max and min muscle activation from the yaml file
    with open(muscleActivationCalibrationFile, 'r') as file:
        # Load the yaml file
        calibrationData = safe_load(file)

        # Get the max and min muscle activation
        maxMuscleActivation = np.array(calibrationData['maxMuscleActivation'])
        minMuscleActivation = np.array(calibrationData['minMuscleActivation'])

    return EmptyResponse()

# Service to get the min activation data for all muscles at once
def getAllMinActivation(request: Empty):
    global muscleActivationHistory, minMuscleActivation

    # Get the min activation data for all muscles
    for i in range(8):
        # Set the min muscle activation to the current muscle activation
        minMuscleActivation[i] = np.average(muscleActivationHistory[i])

    # Write the min muscle activation to the yaml file
    with open(muscleActivationCalibrationFile, 'w') as file:
        # Write the calibration data to the yaml file
        file.write('maxMuscleActivation: ' + str(maxMuscleActivation.tolist()) + '\n')
        file.write('minMuscleActivation: ' + str(minMuscleActivation.tolist()) + '\n')

    return EmptyResponse()

#* ------------------------- Parameter server callback ------------------------ #
# Dynamic reconfigure callback
def dynamicReconfigCallback(config, level):
    # Getting global variables
    global calibrationPeriodDyn, windowSizeDyn, stiffnessMethodDyn, curMuscleDyn, muscleActivationWindowSizeDyn

    # Get and set the calibration period
    calibrationPeriodDyn = config['calibration_period']

    # Get and set the window size
    windowSizeDyn = config['EMG_window_size']

    # Get and set the stiffness method
    stiffnessMethodDyn = config['stiffness_method']

    # Get and set the current muscle
    curMuscleDyn = muscleToEMG[config['cur_muscle']]

    # Set the muscle activation window size
    muscleActivationWindowSizeDyn = config['muscle_activation_window_size']

    # Change the size of the muscle activation history and EMG readings deques
    for i in range(8):
        muscleActivationHistory[i] = deque(maxlen=muscleActivationWindowSizeDyn)
        emgReadingsList[i] = deque(maxlen=windowSizeDyn)

    return config

def main():
    # Initialize the node
    rospy.init_node('trigno_joint_reader')

    #? -------------------------------- Subscribers ------------------------------- #
    # EMG subscriber
    rospy.Subscriber('/X2_SRA_A/trigno_emg', trignoMultiEMG, emg_callback)

    # IMU joint state subscriber
    rospy.Subscriber('/imu_joint_states', JointState, joint_state_callback)

    #? --------------------------------- Services --------------------------------- #
    # Calibration service
    rospy.Service('emgReader/calibrate_EMG', Empty, calibrateEMGCallback)

    # Service to set the max muscle activation
    rospy.Service('emgReader/set_max_muscle_activation', Empty, setMaxMuscleActivation)

    # Service to set the min muscle activation
    rospy.Service('emgReader/set_min_muscle_activation', Empty, setMinMuscleActivation)

    # Read the calibration data from the yaml file
    rospy.Service('emgReader/read_calibration_data', Empty, readCalibrationDataCallaback)

    # Get min activation data for all muscles at once
    rospy.Service('emgReader/get_all_min_activation', Empty, getAllMinActivation)

    #? ---------------------- Start the parameter server ---------------------- #
    dynamic_reconfigure.server.Server(EMGReaderConfig, dynamicReconfigCallback)

    rospy.spin()

if __name__ == '__main__':
    main()
