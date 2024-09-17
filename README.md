# Masters of Robotics capstone project
Author: Zachary Alves

[Portfolio post](https://alves-zach.github.io/projects/01Exoskeleton/)

This project is an extension of the work of [Emek Barış Küçüktabak](https://github.com/emekBaris). I have extended the functionality to accept joint state data from trigno IMU/EMG sensors. The joint state values calculated from the IMU sensors are used as the set points for the exoskeleton controller and the EMG values are used to augment the proportional gain of the controller based on the amount the user is flexing.

[Link to Emek's paper](https://arxiv.org/pdf/2307.06479)

Repositories used:

[multirobot_interaction](https://github.com/Alves-Zach/multi_robot_interaction/tree/main) [main]\
[CANOpenRobotController](https://github.com/emekBaris/CANOpenRobotController/tree/devel/targetVisualization) [devel/targetVisualization]\
[trigno_capture](https://github.com/emekBaris/trigno_capture) [main]\
[trigno_msgs](https://github.com/emekBaris/trigno_msgs) [main]

Some of these repositories are private and the link may not work correctly

## Nodes overview

![Node flowchart](/images/ROSflow.png "Node flowchart")
The layout of the nodes used in this project (x2_dyad is from the [multirobot_interaction](https://github.com/emekBaris/multi_robot_interaction) package written by Emek Barış Küçüktabak)

1. **trignoJointReader.py**: Reads quaternions from trigno_capture node and converts them to hip and knee joint angles

2. **emgReader.py**: Reads EMG data from trigno_capture node and converts them into muscle activation values to augment the controller stiffness

### trignoJointReader.py:

&emsp; Using the quaternions output by the trigno sensors, calculates the angle of each hip and knee joint in the sagittal plane.

### emgReader.py

&emsp; Stiffness calculation methods:

&emsp;&emsp;&emsp; - High-Low method: 

&emsp;&emsp;&emsp;&emsp;&emsp; ![High Low equation](/images/HighLow.png "High Low equation")

&emsp;&emsp;&emsp;&emsp;&emsp; Where σ is an array containing maximum voluntary contraction (MVC) readings for both muscles that control that joint

&emsp;&emsp;&emsp; - Maximum method: 

&emsp;&emsp;&emsp;&emsp;&emsp; Takes the maximum of the two MVC readings and converts that directly to the stiffness multiplier for the controller.

&emsp;&emsp;&emsp; - Minimum method:

&emsp;&emsp;&emsp;&emsp;&emsp; Takes the minimum of the two MVC readings and converts that directly to the stiffness multiplier for the controller.

## Launch file
**exoController.launch**: Launches the following nodes

&emsp;&emsp; trignoJointReader

&emsp;&emsp; emgReader

&emsp;&emsp; multi_robot_interaction

&emsp;&emsp; rqt_gui

## Setup steps

The setup requires a windows PC running the proprietary trigno capture software and nodes used to publish that data as a ROS topic, a router to connect the windows computer to the linux computer running the rest of the nodes, and a CAN adapter that is used to connect to the exoskeleton.

Trigno sensor locations
![Trigno sensor locations](/images/IMUPlacementDiagram.png "Trigno sensor locations")
Above are the locations of the trigno sensors, each oval is one sensor, the color of each oval describes what the data that sensor publishes is used for.

**Linux computer:**
1. Run these commands to set the IPs correctly
```
    set ROS_MASTER_URI=http://174.16.0.1:11311/
    set ROS_IP=174.16.0.2
    set ROS_HOSTNAME=174.16.0.2
```
2. Run `roslaunch CORC x2_real.launch` to connect to the exoskeleton
3. Run three calibration services while the exoskeleton is suspended\
&emsp; - `/X2_SRA_A/calibrate_imu`\
&emsp; - `/X2_SRA_A/calibrate_force_sensors`\
&emsp; - `/X2_SRA_A/start_homing` (This service will make each joint move forward one at a time to the joint limit, ensure the exo can freely move before calling this service)
4. In X2_SRA_A section in rqt_gui ensure these options are selected\
&emsp; - Controller mode 8
&emsp; - Experiment mode 1
&emsp; - Enable each motor with the corresponding check box
5. Put on the exoskeleton
6. Ensure pressure pads are working to determine which foot has more weight on it currently
7. Run `roslaunch imu_exo_control exoLaunch.launch`
8. Put on tringo sensors in locations specified above
9. Calibrate all EMGs\
&emsp; - Have the user flex each muscle that is used in stiffness calculation one by one and call `/emgReader/set_max_muscle_activation` for each muscle, changing cur muscle to choose the next muscle to calibrate, this records all maximum muscle activation\
&emsp; - Have the user sit in a chair, relaxing all muscles then call `/emgReader/get_all_min_activation` to record all minimum muscle activation
10. Optionally rqt_multiplot can be launched as well to help debug or view data, multiple layouts have been provided in */imu_exo_control/rqtLayouts*

**Windows PC:**

1. In a ros command prompt run the following commands to setup ROS IPs correctly
```
    set ROS_MASTER_URI=http://174.16.0.1:11311/
    set ROS_IP=174.16.0.2
    set ROS_HOSTNAME=174.16.0.2
```
2. Using the trigno control utility that can be downloaded [here](https://delsys.com/support/software/), connect 8 sensors
3. Download and build the [trigno_capture](https://github.com/emekBaris/trigno_capture) package written by Emek Barış Küçüktabak
4. Run `roslaunch trigno_capture trigno_capture` to start the publishing of raw IMU and EMG data