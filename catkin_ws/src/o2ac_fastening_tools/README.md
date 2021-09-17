# Introduction
This package controls our fastening tools which are driven by an XL320 motor. 
It also controls the suction and ejection mechanisms.

## Available ROS services
There are two action services.

```
screw_tool_control
```

There is also a topic published for every tool.

```
screw_tool_m4/screw_suctioned
```

Please read 'How to use' for details. 

# Initial Setup
## Dynamixel SDK

The Dynamixel SDK needs to be installed to control the XL320 motors.

This package depends on 'dynamixel-workbench' and 'dynamixel-workbench-msgs'. If you have any trouble building, try this:
```bash
apt-get update && apt-get install -y --no-install-recommends ros-kinetic-dynamixel-sdk
```

## U2D2

For enabling the connection of the U2D2 controllers to a named port, please refer [here](https://gitlab.com/o2ac/ur-o2ac/blob/develop/udev-rules.md).

## ID of XL-320

The motor id needs to be set to move a motor. The ID has to be unique for each Dynamixel motor.

Execute 'motor_id_set.launch'.
The information of a connected motor is displayed.
Through the input screen of the ID setting node, you can set a new ID.

Take care not to have the same IDs for Dynamixels connected to the same controller. You may face communication issues or may not be able to search when IDs overlap.

Specifically, refer to [here](http://support.robotis.com/en/product/actuator/dynamixel_x/xl_series/xl-320.htm#Actuator_Address_03).

Enter the names of each motor ID and gripper in this file: `config/fastening_tools.yaml`

# How to use
## About action service

### screw_tool_control

The action topic is "screw_tool_control" and the action is defined in o2ac_msgs.

- Load gripper info file. (fastening_tools.yaml)
To use the action service, use "fastening_tool_name" and "speed" or "duration" and "direction".
The return value is a boolean type.
The feedback value is current the rotation speed (uint32).

- "fastening_tool_name" : 
The tool names are set in the `fastening_tools.yaml` in the config folder.
If you want to add a new motor, add it to this file.
Do not duplicate the motor names or ids.

- "direction" : 
The valid strings for "direction" are "tighten" or "loosen".

- "speed" : 
When the "direction" is "tighten" mode, this will rotate until the motor stops rotating.
When the "direction" is "loosen" mode, this will rotate until the duration.
The "speed" parameter is mandatory for "tighten" and "loosen" mode.
It can be between 0 and 1023.

- "duration" : 
When the "direction" is "loosen" mode, this will rotate for the specified time.
The "duration" parameter is required for the direction "loosen".
The "duration" is in seconds (ex. "duration" = 1.0 : one second) 
In this mode, there is no feedback.

### suction_control

The action topic is "suction_control" and the action is defined in o2ac_msgs.

load ur_control info file. (suction_control.yaml)

This action is used when you want to execute suction on/off.
If this action is insert screw or trying to break vacuum, if it does not get the result after 5 seconds, it will return 'False' and exit.

To use the action service, use "fastening_tool_name" and "suction_control" and "switch".
The return value is the boolean type.
The feedback value is current the digital_input of ur_controller states of boolean type.

- "fastening_tool_name" : 
The gripper name is set with suction_control.yaml in the config folder.
In this file, the name of screw fastening tool and other info are listed.
For details, I think that you can understand by looking at suction_control.yaml

- "turn_suction_on" :
This value is used to turn the suction on and off via the digital output pin of ur_control.

- "eject_screw" :
This value is used to turn the ejection on and off via the digital output pin of ur_control. If this is not set to True, the screw might stay attached to the tool for a few seconds after suction is turned off. Both turn_suction_on and eject_screw should be set to False after a screw was ejected.

This action service uses the service 'b_bot/ur_hardware_interface/set_io' of the sub module ur_robot_driver.
Please be careful if you use 'b_bot/ur_hardware_interface/set_io' as an alias.

## About Publish screw_suctioned

This topic in suction_controller.py.
This topic publishes suction success for each screw tool as a std_msgs/Bool message, on a topic such as 

```
screw_tool_m4/screw_suctioned
```

This topic uses the publish 'ur_hardware_interface/io_states' of the sub module ur_robot_driver.
Please be careful if you use 'ur_hardware_interface/io_states' as an alias.


## About the actual launch file

As in fastening_tools.launch, set parameter.

'num_controllers' is the number of U2D2 connections.

'serial_port' is set as many set by 'num_controllers'. 
The value uses the connection name of U2D2.

'conf_dir' sets the path of the config folder.

'fastening_tools' sets the yaml name with gripper information.


## Example of use
### fastener_gripper_control 

Appoint gripper name and rotary speed in 'fastener_gripper_action_client.py'.

Please connect U2D2 and XL320 to PC and run 'demo.launch'.

Two XL320 will be used this time.
ID is 1,2 respectively.

I think that the following operation is possible.

- 1. Turn on the torque of XL-320 and rotate clockwise (wheel mode)
- 2. Stop rotation of the motor with fingers, generate load.
- 3. When the load is detected (rotation speed = 0), the torque of XL-320 is turned off and the rotation stop
- 4. To next motor id .


### screw_control

Please run 'suction_demo.launch'.


