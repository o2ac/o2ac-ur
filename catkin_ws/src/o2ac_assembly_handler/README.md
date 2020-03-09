# Assembly Handler package

This package contains assembly definitions for planning in the moveit planning scene, and functionality to load the assemblies.

## Package, File Structure

The structure and names of directories/files is expected to be like this by the package:

```bash
o2ac_assembly_handler                # package directory
│  
├── src/o2ac_assembly_handler        # python package for loading/storing assemlies and publishing them to tf
│   │  
│   ├── assy_reader.py               # python module for internal use (in assy.py) to read the assembly configurations
│   ├── assy.py                      # python module that provides the functionality of the package
│  
├── config                           # directory containing config info for the assemblies
    │  
    ├── assy_1                       # directory for the config info of assembly 'assy_1'
    │   │   
    │   ├── meshes                   # directory for the mesh files of the parts in the assembly
    │   │   │  
    │   │   ├── PART_1_MESH_NAME.stl # mesh file of part 'PART_1_NAME'
    │   │   │   ...
    │   │   ├── PART_N_MESH_NAME.stl # mesh file of part 'PART_N_NAME'
    │   │  
    │   ├── object_metadata          # directory for the extra information associated with the parts
    │   │   │  
    │   │   ├── extra_frames.csv     # csv file description of the subframes of the parts
    │   │  
    │   ├── frames_to_mate.csv       # csv file descriprion of the mating of parts to form an assembly
    │   ├── parts_list.yaml          # yaml file for the list of parts of the assembly
    │...
    ├── assy_n                       # directory for the config info of assembly 'assy_n'
```

### assy_reader.py

This script is for internal use. It defines a class `AssyReader`, that is used inside assy.py

It is used for reading the configuration files for a selected assembly, returning a python list of [`CollisionObjects`](http://docs.ros.org/api/moveit_msgs/html/msg/CollisionObject.html) created from the parts in the assembly, and to return a `tf` tree describing the assembly.

### assy.py

This scipt provides the functionality of the package. It defines a class `AssyHandler` that can be imported in other python modules by:
```python
from o2ac_assembly_handler.assy import AssyHandler
```
after building the package. The `AssyHandler` class provides the following functionality:

By calling the contructor and the `change_assembly` method, an assembly can be selected by name (the name must match one of the direct subdirectories of `config`). The `tf` tree representation of the assembly can be retrieved by reading the `assembly_tree` property of the object. The list of `CollisionObject` representation of the parts of the assembly can be retrieved reading the `collision_objects` property of the object.

The currently stored assembly tree can be published to `tf` by calling the `publish_target_frames` method of the object. The method also takes an optional argument `assy_pose` (of type [`PoseStamped`](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html) in which the desired pose of the assembly can be defined.

The method `lookup_frame` can be used to get the name of a `tf` frame for a part by providing the ID of the `CollisionObject`

### meshes

The name of the mesh files must be provided in the `cad` section of `parts_list.yaml` for each corresponding part.

### extra_frames.csv

This file contains the definition of subframes for the parts in the assembly. The subframes must be defined relative to the part. The name `extra_frames.csv` and `,` separator is expected. For further info, see the header of the file.

### frames_to_mate.csv

This file contains the descriprion of the mating of parts to form an assembly. The mating is defined betwwen the subfrmaes of two parts. Be careful not to over or underdefine the mating because in those cases, the assembly cannot be represented as a valid tree. The name of the file and the names of the frames for mating are expected as in `assy_1/frames_to_mate.csv`.

### parts_list.yaml

This file contains the list of parts of the assembly. The `id`, `name`, and `cad` fields are expected. The `id` and `name` have to be unique.

## Example

For an example see `example_assembly_handler.py` in the `o2ac_examples` package.

### Example usage

In order to use the example, first launch the basic planning scene (```roslaunch o2ac_moveit_config demo.launch``` ) and then run `example_assembly_handler.py` The example should populate the planning scene with collision objects in the arrangement defined in `assy_1` config. The assembly is placed in the `workspace_center` frame. Adding `tf` visualization to RViz should show the subframes of the parts as well. Be careful to exclude the assembly urdf from the base scene, by commenting the line (29) `<xacro:include filename="$(find o2ac_scene_description)/urdf/2019-12-insertion-test-scene.xacro" />` in `o2ac_scene_description/urdf/base_scene.urdf.xarco`, because the same name of the assembiles may cause unexpected behaviour (unless the assembly is not called `assy` it shouldn't cause a problem).
