# Swapping End Effectors
Instructions for swapping between end-effectors.

## Hardware
Unplug the cables attached to the end-effector. There should be **3** cables:
- **12V via the Anderson blade connector** for the linear actuator
- **USB 3.2** for the D405 realsense camera
- **USB 3.2** for the data logger

Unscrew the four screws attaching the end effector to the arm.

Install the alternate gripper by fastening the screws and plugging in the cables.

## Software

### FSM
In `NiMo-FSM/config/default.yaml` change the arguments:
- `replacement` - Manual or automatic replacement of the sensor
    - `'manual'` - Manual replacement of the sensor
    - `'auto'` - Automatic replacement of the sensor
- `clean_extend` - Whether to extend the linear actuator for cleaning and calibrating
    - `True` - Extend the linear actuator to expose the sensor 
    - `False` - Do not extend the linear actuator for cleaning

## Maniuplation
In `NiMo-Manipulation/config/default.yaml` change the arguments:
- `approach` - The method of approach for grasping the cornstalk
    - `'left'` - Approach from the left (hook motion)
    - `'front'` - Approach from the front

In the file `xarm_ros/xarm6_moveit_config/launch/realMove_exec.launch`, add or modify the static transforms for the `gripper` and `camera_link` at the end of the file. They should be in the format:

```
<node pkg="tf" type="static_transform_publisher" name="gripper" args="x y z rz ry rz /link_eef /gripper 100"/>
```