#!/bin/bash

RUN="rosrun o2ac_pose_distribution_updater best_scores"

mesh_files=(
    /root/o2ac-ur/catkin_ws/src/o2ac_assembly_database/config/wrs_assembly_2020/meshes/03-PANEL2.stl
    /root/o2ac-ur/catkin_ws/src/o2ac_assembly_database/config/wrs_assembly_2020/meshes/04_37D-GEARMOTOR-50-70.stl
    /root/o2ac-ur/catkin_ws/src/o2ac_assembly_database/config/wrs_assembly_2020/meshes/05_MBRFA30-2-P6.stl
)

data_files=(
    /root/o2ac-ur/catkin_ws/src/o2ac_assembly_database/config/wrs_assembly_2020/object_metadata/panel_motor.yaml
    /root/o2ac-ur/catkin_ws/src/o2ac_assembly_database/config/wrs_assembly_2020/object_metadata/motor.yaml
    /root/o2ac-ur/catkin_ws/src/o2ac_assembly_database/config/wrs_assembly_2020/object_metadata/motor_pulley.yaml
)

grasp_names=(
    default_grasp
    grasp_1
    grasp_1)

$RUN ${mesh_files[2]} ${data_files[2]} ${grasp_names[2]} 1 1 1 1 1 0.00001 0.001
