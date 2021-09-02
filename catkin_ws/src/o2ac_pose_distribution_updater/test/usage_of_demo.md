1. Launch moveit demo and distribution update server in separeted terminal:
```
roslaunch o2ac_moveit_config demo.launch
roslaunch o2ac_pose_distribution_updater distribution_updater.launch
```

1. Spawn a panel_bearing by
```
rosrun o2ac_routines spawn_panel.py
```
and move to arbitrary pose above the tray. The pose after it is placed to the tray from its current position is set as initial pose.

1. Execute `demo.sh`

1. After the initial pose is visualized, type the goal pose of the panel.
`any` means any pose is accepted, `placed` means that it must be placed finally and grasp name (`default_grasp` or from `grasp_1` to `grasp_28`) means that it must be grasped finally with indicated pose.

1. Wait for planner to finish planning.

1. If the planning has been done successfully, the motion of the plan is visualized.
