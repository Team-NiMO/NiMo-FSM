# NiMo-FSM
A ROS-integrated finite state machine to facilitate the measurement of nitrate levels in cornstalks.

## Overview
### NiMo - Autonomous <ins>Ni</ins>trate <ins>Mo</ins>nitoring in Cornstalks
Nitrate based fertilizer is often over-applied to cornfields leading to detrimental environmental and health impacts. We are working to develop a system to monitor in-planta nitrate levels using a [novel sensor from Iowa State University](https://pubs.acs.org/doi/full/10.1021/acsami.2c01988). More information can be found [our website](https://mrsdprojects.ri.cmu.edu/2024teamd/).

We have split this system into four subsystems:
- [Perception](https://github.com/Team-NiMO/NiMo-Perception_v2) - Localize cornstalks and determine their width
- [Manipulation](https://github.com/Team-NiMO/NiMo-Manipulation) - Control the arm to accomplish system tasks
- [External Mechanisms](https://github.com/Team-NiMO/NiMo-ExternalMechanisms) - Dispsense the cleaning and calibration solution
- [End Effector](https://github.com/Team-NiMO/NiMo-EndEffector) - Control the insertion and reading of the nitrate sensor
- [Navigation](https://github.com/Team-NiMO/MPC_Amiga) - Control the Amiga mobile base to navigate to sampling locations
- [User Interface](https://github.com/Team-NiMO/NiMo-UI) - Allow the user to input sampling locations and recieve feedback from the system

More information about the communication between the FSM and these subsystems is shown in the figure below:

<img src="https://github.com/Team-NiMO/NiMo-FSM/blob/main/docs/fsm.drawio.png" width="650">

The overall flow of the FSM is motivated by the functional architecture shown below. The FSM is broken into six states:
- `Navigation` - Navigate to next waypoint or delta step depending on cornstalk success
- `Finding_Cornstalk` - Detecting cornstalks in the area and selecting one to determine the optimal insertion side
- `Cleaning_Calibrating` - Cleaning and calibrating the sensor to calibrate and check sensor functionality 
- `Insertion` - Grasping the cornstalk, inserting the sensor, and taking readings from nitrate sensor

<img src="https://github.com/Team-NiMO/NiMo-FSM/blob/main/docs/IntegrationFunctional.drawio.png" width="650">

## Installation
Create a new ROS workspace and follow the installation instructions for all subsystems ([Perception](https://github.com/Team-NiMO/NiMo-Perception_v2), [Manipulation](https://github.com/Team-NiMO/NiMo-Manipulation), [External Mechanisms](https://github.com/Team-NiMO/NiMo-ExternalMechanisms), [End Effector](https://github.com/Team-NiMO/NiMo-EndEffector), [Navigation](https://github.com/Team-NiMO/MPC_Amiga), [User Interface](https://github.com/Team-NiMO/NiMo-UI)). If desired, also source the workspace in `~/.bashrc`.
```
mkdir ~/nimo_ws && mkdir ~/nimo_ws/src
echo "source ~/nimo_ws/devel/setup.bash" >> ~/.bashrc
...
```

Next, clone the repository into the `src` folder of your ROS workspace and make the workspace.

```
cd nimo_ws/src
git clone git@github.com:Team-NiMO/NiMo-FSM.git
cd ../ && catkin_make
```

Install ROS SMACH and terminator
```
sudo apt-get install ros-noetic-smach-ros
```

Finally, update the [configuration file](/config/default.yaml) if necessary.

## Use

In separate terminals, run each of the following commands. If the workspace has not been sourced in the `~/.bashrc` you will have to manually source it for each terminal.

```
roslaunch nimo_perception StalkDetect.launch
roslaunch nimo_manipulation nimo_manipulation.launch
roslaunch nimo_end_effector nimo_end_effector.launch
rosrun act_pump actp.py
rosrun nimo_ui ui.py
roslaunch mpc_amiga_mrsd mpc_amiga_mrsd.launch fresh_start:=1
rosrun NiMo-FSM FSM.py
```

## Visualization
Launching the system also starts two visualization tools:
- RViz - Displays the arm configuration, camera feed, and perception predictions
- RQT_Multiplot - Displays the reading of the nitrate sensor in mV

More information about these is detailed in the [Perception](https://github.com/Team-NiMO/NiMo-Perception_v2), [Manipulation](https://github.com/Team-NiMO/NiMo-Manipulation), and [End Effector](https://github.com/Team-NiMO/NiMo-EndEffector) repositories.

## Other
### Swapping End Effector
For swapping end effectors, refer to [swap.md](/docs/swap.md).

## Common Issues
**General Errors**

All errors should appear on the FSM terminal, if the error is not with the FSM itself, determine which subsystem is causing an error and refer to the errors in that terminal for more details.

**Serial Port Issues**

Since both the external mechanisms and end effector use serial communication, if the communication is interrupted, the port may change. Launching the Arduino application and checking the ports can determine which device is linked to which port. More detailed instructions can be found in the respective repositories ([External Mechanisms](https://github.com/Team-NiMO/NiMo-ExternalMechanisms), [End Effector](https://github.com/Team-NiMO/NiMo-EndEffector)).

## Acknowledgements
- Dr. Oliver Kroemer for his assistance and advice
- Dr. George Kantor for his assistance and advice
- [Mark (Moonyoung) Lee](https://github.com/markmlee) for his assistance and advice