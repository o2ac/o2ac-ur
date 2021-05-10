# Defining playback sequences manually in ROS

### parameters
- *robot_name*: Define the name of the robot that will execute this trajectory
- *waypoints*: list of points
    - *pose*: depends on the type joint-space*:array[6] (for ur robots) task-space*:array[7] or array[6]
    - *type*: 'joint-space', 'joint-space-goal-cartesian-lin-motion', 'task-space', 'relative-tcp' or 'relative-base'
    - *speed*: speed scale factor (percentage)
    - *acceleration*: acceleration scale factor (percentage)
    - *blend*: specified only if you want two or more points to be considered as one trajectory with blended intersections
    - *gripper-action*: 'open', 'close' or 'close-open'. Defines the action to take with the gripper, can not be part of a blended trajectory. **It is always executed after reaching the defined posed!**
