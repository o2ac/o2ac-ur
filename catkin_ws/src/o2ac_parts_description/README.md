# o2ac_parts_description

THIS PACKAGE IS MOSTLY DEPRECATED.

--- 

This package includes URDF and mesh models of the parts used in WRC2018. 
The URDF files are generated automatically from the mesh files, and additional frames appended to it.
The models can be used for simulation. You can check the parts are correctly 
spawned in the Gazebo simulation environment by following the steps below.

1. Launch simulated environment in Gazebo
    ```bash
    # In the container
    source ~/catkin_ws/devel/setup.bash
    roslaunch o2ac_gazebo o2ac_gazebo.launch
    ```
2. Run the script for test
    ```bash
    # 7 is the part ID, 5 is the number of parts to spawn, and 1 is the bin ID
    rosrun o2ac_parts_description test_spawn_parts.py 7 5 1
    rosrun o2ac_parts_description test_spawn_parts.py 8 10 2
    rosrun o2ac_parts_description test_spawn_parts.py 9 50 3
    rosrun o2ac_parts_description test_spawn_parts.py 16 50 4
    ```

To view all the parts in Rviz, simply include "display_all_parts.srdf" in the scene.

## Generating the URDF files

Before running the assembly task, the meshes need to be put into the `/meshes` directory of this package, with the first two letters being the part number. Afterwards, execute these commands:

    ```bash
    rosrun o2ac_parts_description generate_urdf_from_meshes.py
    rosrun o2ac_parts_description generate_assembled_piece.py
    ```

The two nodes take the files in `/meshes` and  `/urdf/templates` as input, and generate the URDF files in the `/urdf/generated` directory. Files are overwritten, so be careful.

To change the assembly and the named frames, change the csv files in `/urdf/templates/`.

## Rules for item coordinate systems

Meshes should be defined such that the coordinate system lies in an easily understood point on the object (in a corner, on the axis of symmetry at the end of the part etc).

## How to define "mating" frames

We plan to define frames to be used for planning motions on all of the parts. Thoughts:

- Align the x-axis with the axis of rotation
- Let the x-axis point inside holes
- Let the x-axis point away from male parts (cylinders)
- Put the frame at the surface level of surrounding material for holes, and at the tip of cylinders\screws
- For male parts, the frame at the tip should be called _tip, the second one _inserted (the latter one will be fixed to the _hole frame)
- For parts that are both rotation symmetric and can be turned over (washers, nuts, spacers...) only one frame should be in the center
