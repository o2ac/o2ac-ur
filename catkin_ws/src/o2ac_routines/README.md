# Introduction

This node contains routines using high-level actions. It is meant to enable quick prototyping and to represent task instructions in a simple and compact way.

# Structure

- base.py offers convenience functions that access other services and actions, and move the robot
- taskboard.py and assembly.py should contain the full set of instructions for each task
- calibration.py should be used to check the calibration for each scene (e.g. move the robots to the corners of a container or the assembly)

# Notes

This Python script is complementary to the actions provided by o2ac_skill_server.
