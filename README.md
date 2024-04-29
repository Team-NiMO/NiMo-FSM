# NiMo-FSM
This node handles the transition between different states based on the functional architecture which we have defined for our system [Fig. FA]. Services are defined for each functionality for the respective nodes and the FSM script calls each service based on the system transition. [Fig. FSM]

## Overview
[LINKS TO MANIUPLATION, PERCEPTION, EXTERNAL MECHANISMS, END EFFECTOR]
Fig 1. FUNCTIONAL ARCHITECTURE
<img src="https://github.com/Team-NiMO/NiMo-FSM/docs/FSM.png" width="650">

Fig 1. SERVICE CALLS
<img src="https://github.com/Team-NiMO/NiMo-FSM/docs/fsm.drawio.png" width="650">

## Installation
'''
git clone git@github.com:Team-NiMO/NiMo-FSM.git
git pull
'''
- python version >= 3
- install SMACH -
'''
sudo apt-get install ros-noetic-smach-ros
'''

[ROS PACKAGES?]
[SETUP FOR TERMINATOR LAUNCH]git@github.com:Team-NiMO/NiMo-FSM.git

## Launching FSM
'''
cd nimo_ws
rosrun NiMo-FSM FSM.py
'''

[LAUNCHING EVERYTHING W/ TERMINATOR]
[LAUNCHING EVERYTHING W/O TERMINATOR]

## Visualization
[EXPLANATION OF EVERYTHING]

[RVIZ IMAGE]

## Other
### Future improvements
[???]

## Common Issues
1. If the terminator is not killed properly, i.e. if all the nodes are not killed properly, then there might be arduino port issues. The possible solutions are - 
a. Checking the port number assigned and then modifying actp.py script for arduino uno and nitrate_sample.py for bluefruit nano
NOTE: Restarting the amiga system will reassign the previous port numbers, so always check the port numbers before starting the FSM

2. Since Service calls are blockers, FSM wont start unless all the nodes are launched, because it will wait for all the services to start. So always launch all the nodes before starting the FSM.

## Acknowledgements
FILES TO INCLUDE: Terminator config, RViz file
