# Introduction

This package contains:

1) (in `scripts`) executable scripts to run the competition tasks and calibration/test routines
2) (in `src`) Python classes to control the complete robot system (robots, tools, vision)

# Executable scripts

- `taskboard.py` and `assembly.py` run the competition tasks
- `calibration.py` and `osx_view_testing.py` are calibration and convenience testing routines 
- `show_assembly.py` visualizes the different assemblies in the database
- The remaining files (e.g. `test.py`) are for miscellaneous tests and scrap code

# "Controller" classes

- `common.py` extends `base.py` and offers a class to control the entire robot system. This class (`O2ACCommon`) owns other classes that control different subsystems:
    - The robot arms (via `robot_base.py`, `ur_robot.py`, `dual_arm.py`, `robotiq_gripper.py` and `ur_force_control.py`)
    - The vision system (via `vision_client.py`)
    - The screw tools (via `tools.py`)
- `taskboard.py` and `assembly.py` extend `O2ACCommon` with task-specific variables and functions

# Movement Sequences

We implement command sequences, in which the next motion is either planned while the previous motion is being executed, or the joint trajectories can be saved/loaded to/from a file, to be executed without requiring any planning time.

For an example, see the `subtask_zero` method in `src/o2ac_routines/assembly.py`. For the implementation, see the relevant sections `base.py` (functions related to `execute_sequence`).

Also see [pilz_robot_programming](https://github.com/PilzDE/pilz_industrial_motion/tree/melodic-devel/pilz_robot_programming) for an alternative Python implementation, which is probably cleaner, and from which we took inspiration.

# Calibrations

TODO: Define a calibration procedure for automatically identify the relative position of the robots with respect to each other an the 
table.
TODO: Suggestion (cambel): 
- Design a simple end-effector for the robots identical for each robot with a small tip. This should be attached instead of the grippers.
- Design a board with holes that match the small tip of the end-effector.
- Compute the relative offsets between the robots by manually moving the robot to the poses defined in the board and recording those poses. 

For now, this is the recommended MANUAL procedure:

  0. In case of moving the robot bases (tables). Start by leveling the robots with respect to the table. Use the TeachPendant to move the robot right to left, front to back with the gripper barely at the level of the table to make sure that the robots are level.
  1. Use `calibration.py > 501 & 502`, you need to choose a base plate and then load the corresponding database before using this options. E.g. `load2021` then `501`. This will help you define the offsets of the robots with respect to a fix point (in this case the holes on the base plate). **TIP**: put a piece of metallic ruler or some very straight plane in the center of the grippers to make it easier to visually inspect the offset errors. Do the same with a 90deg rotated position of the gripper to confirm the offsets on the perpendicular direction. Then update the offsets in `catkin_ws/src/o2ac_scene_description/urdf/base_scene.urdf.xacro`. You may need to update the rotation offset around **axi-z**
  2. Use  `calibration.py > bm4 & am4` to calibrate the position of the tools with respect to the robot. Height and y-position. Then update the offsets in `catkin_ws/src/o2ac_scene_description/urdf/components/tools.xacro`
  3. For each tool repeat a similar procedure as explain in this point. For this example, let's use the tool M4 with b_bot, equip the tool with b_bot and use the `calibration.py > 504` to finetune the TCP position of the tool. Update the following file `catkin_ws/src/o2ac_scene_description/urdf/components/o2ac_end_effector.urdf.xacro`. For a_bot, update `catkin_ws/src/o2ac_scene_description/urdf/components/o2ac_end_effector_2F-140.urdf.xacro`
