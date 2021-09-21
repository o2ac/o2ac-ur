# Introduction

This package contains the description of our robot system and scenes. The part definitions can be found in o2ac_parts_description. 

# How to change the scene of a certain task (Taskboard, Assembly)

If you want to adjust the positions of parts that are specific to your task, change only the `assembly_scene.xacro` and `taskboard_scene.xacro` files.

The task is set via a parameter on the `base_scene.urdf.xacro`.

# Known issues

Increasing the size of the boxes too much causes them to be unstable in the Gazebo simulation.
