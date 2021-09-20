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
