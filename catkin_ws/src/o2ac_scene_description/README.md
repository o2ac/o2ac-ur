# Introduction

This package contains the description of our robot system and scenes. The part definitions can be found in o2ac_parts_description. 

# How to change the scene of a certain task (kitting, taskboard, assembly)

If you want to adjust the positions of parts that are specific to your task, change only the "kitting_scene.xacro", "assembly_scene.xacro" and "taskboard_scene.xacro" files.

The task will be set via a parameter that is to be announced.

# Known issues

Increasing the size of the boxes too much causes them to be unstable in the Gazebo simulation.
