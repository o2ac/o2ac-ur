# Defining playback sequences manually in ROS

### parameters
- *robot_name*: Define the name of the robot that will execute this trajectory
- *waypoints*: list of points
    - *pose*: depends on the type joint-space*:array[6] (for ur robots) task-space*:array[7] or array[6]
    - *pose_type*: 'joint-space', 'joint-space-goal-cartesian-lin-motion', 'task-space-in-frame', 'relative-tcp' or 'relative-base'
    - *speed*: speed scale factor (percentage)
    - *acceleration*: acceleration scale factor (percentage)
    - *blend*: specified only if you want two or more points to be considered as one trajectory with blended intersections
    - *gripper*: Defines the action to take with the gripper, can not be part of a blended trajectory. **It is always executed after reaching the defined posed!**
    - *linear*: Whether to force linear path during planning
        - *action*: 'open', 'close' or 'close-open'. 
        - *opening_width*: 0.14 only considered for open or close-open actions
        - *force*: from 40 to 100 (N), only on close or close-open actions
        - *velocity*: 0.01 to 0.1 open/close speed

### Types of pose
joint-space # No IK
joint-space-goal-cartesian-lin-motion # Use linear motion to reach pose

task-space-in-frame # Go to specific point in cartesian space w.r.t a given frame. Default frame "world". Orientation RPY given in degrees
    move_linear: whether to use linear motion or no

relative-to-base # Relative motion in robot's base frame. Orientation RPY given in degrees
relative-to-world # Relative motion in scene's world frame. Orientation RPY given in degrees
relative-to-tcp # Relative motion in robot's default TCP frame. Orientation RPY given in degrees

named-pose # Go to a predefined pose with name

### Trajectories
To create trajectories, add the parameter `is_trajectory: True` to the waypoint.
The point will be append to any consecutive waypoint with the same parameter defined
Only supported for pose types `joint-space-goal-cartesian-lin-motion` and `task-space-in-frame`
The speed and acceleration will be taken from the first point
