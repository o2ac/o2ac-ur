# Dynamixel Command package

This package can be used to configure and command Dynamixel motors using an action server.
This is used by `o2ac_fastening_tools` to actuate the tools using Dynamixel motors.

## Setup

The first thing to do is to make the Dynamixel controller (U2D2) available inside the docker container. For this a `.rules` file has to be created in `/etc/udev/rules.d` that defines the binding of the USB device to a name that is defined in the `docker_compose.yml`. Refer to `udev-rules.md` in the wiki for instructions. Note that the Docker container may need to be restarted for the new USB connection to be available: `docker restart your_container`

Once it is done, start the container and make the configurations.

### Set up motors

The motors can be set up with the `motors_config.yaml` file in the `config` directory of the package.
The structure of this configuration file is explained [here](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/) in section 5.1.2.
The action servers expects the `current_limit`, `max_position_limit` and `min_position_limit` values to be set for each motor.

### Set up controller

The controller setup can be done by changing the parameters in the `dynamixel_command.launch` launch file in the `launch` directory of the package. The meaning of the parameters can be found [here](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/) in section 5.1.2.
The `gripper_current_ratio` parameter can be used to set the grasping force (current) rational to the maximum value (`current_limit` in the motor config).
This parameter is applied to all of the motors connected to the controller.

## How to use

After setting up the motors and the controller launch the `dynamixel_command.launch` file. In order to send commands to the motors, create an actionlib
client for the `/controller_ns/action_server` action. The action messages are defined in the `o2ac_msgs/action/DynamixelGripperCommand.action` file.
The result of the action is a boolean value indicating the success of the action. There is no feedback during the action execution.
