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
roslaunch o2ac_assembly_database visualize_object_metadata.launch assy_name:="wrs_assembly_1" object_name:="base"
```

If `object_name` is not specified, all objects of the specified assembly will be visualized. Other arguments:

Other arguments:
 - `only_subframes`: if set to true, only the object(s) and the subframes are visualized (default: false)
 - `only_grasps`: if set to true, only the object(s) and the grasps are visualized (default: false), if `only_subframes` is true this argument has no effect
 - `gripper_at_grasp`: set this to a name of the grasp as displayed in the visualization to visualize the gripper with its tip link at that grasp (default: "") This argument only works when visualizing a single object with `only_subframes` set to false.
 - `assy_name`: The name of the assembly to display. `wrs_assembly_1` by default.

Examples:

Visualize all objects of `wrs_assembly_1` with only their subframes
```
roslaunch o2ac_assembly_database visualize_object_metadata.launch only_subframes:=true
```

Visualize `bearing` object of `taskboard` with subframes, grasps, and arrows and the gripper set to grasp `grasp_2`
```
roslaunch o2ac_assembly_database visualize_object_metadata.launch assy_name:=taskboard object_name:=bearing gripper_at_grasp:=grasp_2
```


## Package, File Structure

The structure and names of directories/files is expected to be like this by the package:

```bash
o2ac_assembly_database                    # package directory
│  
├── src/o2ac_assembly_database        # python package for loading/storing assemblies and publishing them to tf
│   │  
│   ├── parts_reader.py                   # python module for internal use (in assy.py) to read the assembly configurations
│   ├── assy.py                          # python module that provides the functionality of the package
│   ├── visualize_metadata.py            # python module for visualizing the object metadata (subframes and grasps)
│  
├── config                               # directory containing config info for the assemblies
│   │  
│   ├── assy_1                           # directory for the config info of assembly named 'assy_1'
│   │   │   
│   │   ├── meshes                       # directory for the mesh files of the parts in the assembly
│   │   │   │  
│   │   │   ├── 'MESH_NAME'.stl          # (Example mesh file names: 01-BASE.stl, motor_mesh.stl, Panel.stl, ...)
│   │   │   │   ...
│   │   │   ├── 'ANOTHER_MESH_NAME'.stl
│   │   │  
│   │   ├── object_metadata              # directory for the extra information associated with the parts
│   │   │   │  
│   │   │   ├── 'name_of_part'.yaml      # yaml file description of the subframes and grasps of part named 'name of part' (Example names for parts: 'base', 'panel', 'motor' ...)
│   │   │   │  ...
│   │   │   ├── 'name_of_other_part'.yaml
│   │   │  
│   │   ├── frames_to_mate.csv           # csv file description of the mating of parts to form an assembly
│   │   ├── parts_list.yaml              # yaml file for the list of parts of the assembly
│   │...
│   ├── assy_n                           # directory for the config info of assembly named 'assy_n'
│  
├── launch                               # directory containing launch file for the object metadata visualization
│   │  
│   ├── visualize_object_metadata.launch # launch file for the object metadata visualization
│   ├── rviz.launch        # additional launch file for move group, rviz and loading the urdf
│   ├── demo.rviz                        # rviz setup for the visualization
│  
├── urdf                                 # directory containing the urdf of the gripper for visualization
│   │  
│   ├── eef.urdf.xacro                   # end-effector urdf
│   ├── eef.srdf                         # end-effector srdf
```

### parts_reader.py

This script is for internal use. It defines a class `PartsReader`, that is used inside assy.py

It is used for reading the configuration files for a selected assembly, returning a python list of [`CollisionObjects`](http://docs.ros.org/api/moveit_msgs/html/msg/CollisionObject.html) created from the parts in the assembly, and to return a `tf` tree describing the assembly.

### assy.py

This script provides the functionality of the package. It defines a class `AssemblyReader` that can be imported in other python modules by:
```python
from o2ac_assembly_database.assembly_reader import AssemblyReader
```
after building the package. The `AssemblyReader` class provides the following functionality:

By calling the constructor and the `change_assembly` method, an assembly can be selected by name (the name must match one of the direct subdirectories of `config`). The `tf` tree representation of the assembly can be retrieved by reading the `assembly_tree` property of the object. The list of `CollisionObject` representation of the parts of the assembly can be retrieved reading the `collision_objects` property of the object.

The currently stored assembly tree can be published to `tf` by calling the `publish_assembly_frames` method of the object. The method also takes an optional argument `assy_pose` (of type [`PoseStamped`](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html) in which the desired pose of the assembly can be defined.

The method `lookup_frame` can be used to get the name of a `tf` frame for a part by providing the ID of the `CollisionObject`

### visualize_object_metadata.py

This is the node for visualizing the metadata of a given part.

### meshes

The name of the mesh files can be chosen freely, but they have to be provided in the `cad` section of `parts_list.yaml` for each corresponding part.

### 'name_of_part'.yaml

This file contains the definition of subframes and grasps for a part (referred to by name) in the assembly. The subframes and grasps must be defined relative to the part in the same fashion. The subframes are stored as part of the `CollisionObject` messages. The grasps are uploaded to the ROS parameter server and can be accessed with the following naming scheme:

```bash
/'assembly_name'/'object_name'/'grasp_name'
```

All of the grasps for a single object can be retrieved as a dictionary by requesting the parent parameter such as

```bash
/'assembly_name'/'object_name'
```

Requesting only the root parameter `/'assembly_name'` returns all of the grasps for all objects of that assembly as a dictionary.

### frames_to_mate.csv

This file contains the description of the mating of parts to form an assembly. The mating is defined between the subframes of two parts. Be careful not to over or underdefine the mating because in those cases, the assembly cannot be represented as a valid tree. See the header and the contents of the file for instructions on how to define matings between the subframes.

### parts_list.yaml

This file contains the list of parts of the assembly. The `id`, `name`, and `cad` fields are expected. The `id` and `name` have to be unique.
