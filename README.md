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

More information about the communication between the FSM and these subsystems is shown in the figure below:

<img src="https://github.com/Team-NiMO/NiMo-FSM/blob/main/docs/fsm.drawio.png" width="650">

The overall flow of the FSM is motivated by the functional architecture shown below. The FSM is broken into six states:
- `Global` - Check whether it is necessary to navigate to field or continue to next waypoint
    - `global_nav_stat` : True -> Global navigation in progress | False -> amiga already in field
- `Navigation` - Navigate to next waypoint or delta step depending on cornstalk success
    - `found_plan` : True -> Plan has been found | False -> Plan has not been found
    - `more_waypoints` : True -> More waypoints exist | False -> All waypoints have been exhausted
- `Finding_Cornstalk` - Detecting cornstalks in the area and selecting one to determine the optimal insertion side
- `Cleaning_Calibrating` - Cleaning and calibrating the sensor to calibrate and check sensor functionality 
- `Insertion` - Grasping the cornstalk, inserting the sensor, and taking readings from nitrate sensor
- `Replace` - Depending on the end-effector, moving to manually or automatically replace a broken sensor

<img src="https://github.com/Team-NiMO/NiMo-FSM/blob/main/docs/IntegrationFunctional.drawio.png" width="650">

## Installation
Create a new ROS workspace and follow the installation instructions for all subsystems ([Perception](https://github.com/Team-NiMO/NiMo-Perception_v2), [Manipulation](https://github.com/Team-NiMO/NiMo-Manipulation), [External Mechanisms](https://github.com/Team-NiMO/NiMo-ExternalMechanisms), [End Effector](https://github.com/Team-NiMO/NiMo-EndEffector)). If desired, also source the workspace in `~/.bashrc`.
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
sudo apt-get install terminator
```

Move the [terminator config](/docs/config) to the terminator configuration folder. If you already have terminator configs, you will need to merge the `nimo` config with your existing config file.
```
cp docs/config ~/.config/terminator/
```

Finally, update the [configuration file](/config/default.yaml) if necessary.

## Use
There are two methods to launch the system: 
- Manually via multiple terminal commmands
- Automatically via a terminator config
    - This method was chosen over a launch file so that the individual systems could be monitored separately

These methods will launch every node sequentially and display visualization tools mentioned in the next section.

### Launching Manually
In separate terminals, run each of the following commands. If the workspace has not been sourced in the `~/.bashrc` you will have to manually source it for each terminal.

```
roslaunch nimo_perception StalkDetect.launch
roslaunch nimo_manipulation nimo_manipulation.launch
rosrun nimo_end_effector nitrate_sample.py
rosrun act_pump actp.py
rosrun NiMo-FSM FSM.py
```

### Launching Automatically
From specifically terminal (not terminator) run the following commands.
```
pkill terminator
terminator -l nimo
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

**Terminal doesn't load properly**

If terminator is not killed properly, the windows may not load in the correct configuration. If this is the case, kill terminator and restart.

**Serial Port Issues**

Since both the external mechanisms and end effector use serial communication, if the communication is interrupted, the port may change. Launching the Arduino application and checking the ports can determine which device is linked to which port. More detailed instructions can be found in the respective repositories ([External Mechanisms](https://github.com/Team-NiMO/NiMo-ExternalMechanisms), [End Effector](https://github.com/Team-NiMO/NiMo-EndEffector)).

## Acknowledgements
