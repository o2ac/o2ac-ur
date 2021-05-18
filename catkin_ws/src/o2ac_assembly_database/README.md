# Assembly Handler package

This package contains assembly definitions for planning in the MoveIt planning scene, and functionality to load the assemblies.

## Usage in code

For an example of how to use the class in Python, see `example_assembly_reader.py` in the `o2ac_examples` package.

In order to run the example, first launch our robot scene (```roslaunch o2ac_moveit_config demo.launch``` ) and then run `example_assembly_reader.py`. The example should populate the planning scene with collision objects in the arrangement defined in `assy_1` config. The assembly is placed in the `workspace_center` frame. Adding `tf` visualization to RViz should show the subframes of the parts as well. Be careful to exclude the assembly URDF from the base scene, by commenting the line (29) `<xacro:include filename="$(find o2ac_scene_description)/urdf/2019-12-insertion-test-scene.xacro" />` in `o2ac_scene_description/urdf/base_scene.urdf.xarco`, because the same name of the assemblies may cause unexpected behavior (unless the assembly is not called `assy` it shouldn't cause a problem).

## Visualizing metadata

To check if objects and their metadata has been defined correctly, all parts of an assembly can be visualized by first launching:

```
roslaunch o2ac_assembly_database rviz.launch 
```

to start the RViz visualization. Note that the robot scene is not required! and then:

```
roslaunch o2ac_assembly_database visualize_object_metadata.launch
```

to visualize the metadata of the objects. `visualize_object_metadata.launch` has several arguments for modifying its functionality. Example:

```
roslaunch o2ac_assembly_database visualize_object_metadata.launch db_name:="wrs_assembly_2020" object_name:="base"
```

If `object_name` is not specified, all objects of the specified assembly will be visualized. Other arguments:

Other arguments:
 - `only_subframes`: if set to true, only the object(s) and the subframes are visualized (default: false)
 - `only_grasps`: if set to true, only the object(s) and the grasps are visualized (default: false), if `only_subframes` is true this argument has no effect
 - `gripper_at_grasp`: set this to a name of the grasp as displayed in the visualization to visualize the gripper with its tip link at that grasp (default: "") This argument only works when visualizing a single object with `only_subframes` set to false.
 - `db_name`: The name of the assembly to display. `wrs_assembly_2020` by default.

Examples:

Visualize all objects of `wrs_assembly_2020` with only their subframes
```
roslaunch o2ac_assembly_database visualize_object_metadata.launch only_subframes:=true
```

Visualize `bearing` object of `taskboard` with subframes, grasps, and arrows and the gripper set to grasp `grasp_2`
```
roslaunch o2ac_assembly_database visualize_object_metadata.launch db_name:=taskboard object_name:=bearing gripper_at_grasp:=grasp_2
```


## Package, File Structure

```bash
o2ac_assembly_database                   # package directory
│  
├── src/o2ac_assembly_database           # python package for loading/storing assemblies and publishing them to tf
│   │  
│   ├── parts_reader.py                  # Handles the parts and collision object loading
│   ├── assembly_reader.py               # Handles the assembly tree structure and publishing to TF
│   ├── visualize_metadata.py            # For visualizing the object metadata (subframes and grasps)
│  
├── config                               # directory containing config info for the assemblies
│   │  
│   ├── assembly_1                       # directory for the config info of assembly named 'assembly_1'
│   │   │   
│   │   ├── meshes                       # directory for the mesh files of the parts in the assembly
│   │   │   │  
│   │   │   ├── 'MESH_NAME'.stl          # (Example mesh file names: 01-BASE.stl, motor_mesh.stl, Panel.stl, ...)
│   │   │   │   ...
│   │   │   ├── 'ANOTHER_MESH_NAME'.stl
│   │   │  
│   │   ├── object_metadata              # directory for the extra information associated with the parts
│   │   │   │  
│   │   │   ├── 'partA'.yaml             # yaml file description of the subframes and grasps of part named 'partA'
│   │   │   │  ...
│   │   │   ├── 'name_of_other_part'.yaml
│   │   │  
│   │   ├── frames_to_mate.csv           # csv file describing of the mating of parts to form an assembly
│   │   ├── parts_list.yaml              # yaml file for the list of parts of the assembly
│   │...
│   ├── assembly_n                       # directory for the config info of assembly named 'assembly_n'
│  
├── launch                               # directory containing launch file for the object metadata visualization
│   │  
│   ├── visualize_object_metadata.launch # launch file for the object metadata visualization
│   ├── rviz.launch                      # additional launch file for move group, rviz and loading the urdf
│   ├── demo.rviz                        # rviz setup for the visualization
│  
├── urdf                                 # directory containing the urdf of the gripper for visualization
│   │  
│   ├── eef.urdf.xacro                   # end-effector urdf
│   ├── eef.srdf                         # end-effector srdf
```

### parts_reader.py

This script defines the class `PartsReader`, which is used to read in a database of parts, publish them as [`CollisionObject`s](http://docs.ros.org/api/moveit_msgs/html/msg/CollisionObject.html), or convert between their name/id/type.

### assembly_reader.py

This defines the class `AssemblyReader` which offers all the functionality of `PartsReader`, but also allows publishing an assembly tree to TF, which shows all the object frames and their subframes in the fully assembled (target) state.

The assembly can be selected with the `change_assembly` method (the name must match one of the subfolders in `config`). The `tf` tree of the assembly is stored in the class' `assembly_tree` property. The `CollisionObject` for each part of the assembly can be retrieved via `get_collision_object`.

The current assembly tree can be published to `tf` by calling the `publish_assembly_frames` method of the object. 

The method `lookup_frame` can be used to get the name of a `tf` frame for a part by providing the ID of the `CollisionObject`.

### visualize_object_metadata.py

This node visualizes the metadata (grasp poses) of a part. Use `visualize_metadata.launch` to display the metadata of all files in an assembly or parts database.

### meshes

The name of the mesh files can be chosen freely, but they have to be provided in the `cad` section of `parts_list.yaml` for each corresponding part.

### 'name_of_part'.yaml

This file contains the definition of subframes and grasps for a part (referred to by name) in the assembly. The subframes and grasps must be defined relative to the part in the same fashion. The subframes are stored as part of the `CollisionObject` messages. The grasps are uploaded to the ROS parameter server and can be accessed by:

```bash
/'assembly_name'/'object_name'/'grasp_name'
```

All of the grasps for a single object can be retrieved as a dictionary by requesting the parent parameter:

```bash
/'assembly_name'/'object_name'
```

Requesting only the root parameter `/'assembly_name'` returns all of the grasps for all objects of the assembly as a dictionary.

### frames_to_mate.csv

This file contains the description of the mating of parts to form an assembly. The mating is defined between the subframes of two parts. Be careful not to over or underdefine the mating, as the assembly cannot be represented as a valid tree otherwise (there can be no closed loops in a TF tree). See the header and the contents of the file for instructions on how to define matings between the subframes.

### parts_list.yaml

This file contains the list of parts of the assembly. The `id`, `name`, and `cad` fields are expected. The `id` and `name` have to be unique.
