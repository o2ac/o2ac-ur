# moveit_task_constructor_demo

Description: A simple pick & place demo using MoveIt Task Constructor. This uses the Panda from Franka Emika

Developed by Henning Kayser & Simon Goldstein at [PickNik Consulting](http://picknik.ai/)

## Run

Run demo

    roslaunch moveit_task_constructor_demo demo.launch

# mtc_modules_server

This node starts multiple action servers for different tasks, such as `Pick Object`, `Place Object` etc. The action servers do the task planning when a request arrives and they return the best solution if it is found, which contains the trajectories for executing the plan.

## Run

### Requirements

The node requires certain parameters to be set on the ROS parameter server for its initialization. These are used as default values for the tasks' parameters (the parameters that most likely do not change for each call to the action servers, such as the name of the robots, directions for lifting, or placing objects, the name of the support surfaces ...)

Before starting the node make sure that the following parameters are available on the parameter server:

 - `mtc_modules/arm_group_names`: list of strings (list of names for the planning groups of the robot arms)
 - `mtc_modules/hand_group_names`: list of strings (list of names for the planning groups of the grippers)
 - `mtc_modules/support_surfaces`: list of strings (list of names for the support surfaces of the objects)
 - `mtc_modules/grasp_parameter_location`: string (namespace under which grasps are looked for on the parameter server)

 - `mtc_modules/lift_direction_reference_frame`: string (name of the frame in which the lift direction is defined)
 - `mtc_modules/lift_direction`: list of float64 values in the form of [x, y, z] (vector, pointing in the direction of lifting the object)

 - `mtc_modules/approach_place_direction_reference_frame`: string (name of the frame in which the placing direction is defined)
 - `mtc_modules/approach_place_direction`: list of float64 values in the form of [x, y, z] (vector, pointing in the direction of approaching while placing the object)

 - `mtc_modules/retreat_direction_reference_frame`: string (name of the frame in which the retreat direction is defined after placing the object, if left empty it is set to be the end-effector frame)
 - `mtc_modules/retreat_direction`: list of float64 values in the form of [x, y, z] (vector, pointing in the direction of retreating after placing the object)

 These parameters are **read at the initialization of the node only**, so changing the values on the parameter server after the node has been started has no effect. **For a new set of initial parameters the node has to be restarted**.

 When making calls to the action servers, if a message field that has a corresponding parameter among the above is not specified, the value of the parameter is used for the planning (this is how **these parameters are used as default values**). If a corresponding field is specified in the action message, the value in the action message will be used instead, but the value of these parameters persist.

### Grasps

The node relies on a stage that looks for and loads the possible grasps from the ROS parameter server instead of the GenerateGraspPose stage. The grasps on the parameter server are expected to be in the following format:

```
/assembly_level_namespace/object_level_namespace/grasp_level_namespace/position  # A list of three values ([x,y,z])
/assembly_level_namespace/object_level_namespace/grasp_level_namespace/orientation  # A list of four values (quaternion, [x,y,z,w])
```

Getting the `/assembly_level_namespace/object_level_namespace/grasp_level_namespace` parameter enables the retrieval of a single grasp of a single object. The returned value is a dictionary of the form: `{"position":[], "orientation":[]}`

To get all the grasps of a single object the parent parameter can be retrieved, such as `/assembly_level_namespace/object_level_namespace`. This returns a dictionary of dictionaries that contains all the grasps of a single object, like: `{"grasp_1":{"position":[], "orientation":[]}, "grasp_2":{"position":[], "orientation":[]}, ...}`

Retrieving `/assembly_level_namespace` parameter only, returns all of the grasps of all of the objects in this collection.

The `mtc_modules/grasp_parameter_location` parameter refers to the `assembly_level_namespace` and is used to retrieve the correct grasps. The `object_name` field of the action messages refers to the `object_level_namespace`. Please note, that the planners expect the grasps to be loaded to the parameter server and the planning will not be successful if the grasps are not available.

### EEF names and frames

The naming of the eef (defined in the srdf) is expected to follow the pattern:

`eef_name` = `arm_group_name` + `_tip`,

where `arm_group_name` is the name of one of the planning groups of the robot arms (given by the `mtc_modules/arm_group_names` required parameter).

The naming of the eef frames are expected to follow the pattern:

`eef_frame_name` = `hand_group_name` + `_tip_link`,

where `hand_group_name` is the name of one of the planning groups of the grippers (given by the `mtc_modules/hand_group_names` required parameter).

### Moveit

For the planning to work, moveit has to be running. Make sure to start moveit before running the node.

### Running the node

**Before running the node make sure that the required parameters (initial parameters and grasps) are loaded to the parameter server, and Moveit is running.**

To run the node type:

```bash
rosrun moveit_task_constructor_demo mtc_modules_server 
```

If the node is initialized properly and the servers are running a message should be printed to the terminal, similar to this:

```bash
[ INFO] [1597642900.795329846]: Starting MTC Modules action servers
[ INFO] [1597642900.795867665]: Initializing Modules Planner
[ INFO] [1597642900.800049610]: Initialization finished!
Ready to start planning ...
```

## How to use

The actions provided by the node, if it is running correctly, are:

 - `/pick_planning` (picking an object)
 - `/place_planning` (placing an already picked object)
 - `/release_planning` (releasing a held object)
 - `/fastening_planning` (move_above - go_down - come_up while holding an object/tool the whole time)
 - `/pick_place_planning` (picking and placing an object potentially including a regrasp)
 - `/sub_assembly_planning` (picking and placing the L panel onto the base plate and fixing it with the screws)

 To start planning, make requests from an action client to use the servers. An easy way to test them is with the `actionlib` package.

To test the action server for pick planning, type:

 ```bash
 rosrun actionlib axclient.py /pick_planning
 ```

This opens a new window where the action goal can be set and the feedback/result can be inspected. After sending the goal, the planning should start. This can be seen in the terminal in which `mtc_modules_server` is running, or with RViz.

To visualize the task planning with RViz add the `Motion Planning Tasks` display. In the `Motion Planning Tasks` panel, under `Task Tree` the hierarchical representation of the task should be visible if the planning has started. Selecting one of the stages in the task should reveal the solutions of the selected stage on the right side of the `Motion Planning Tasks` panel. By clicking on one of the solutions, the motion plan can be visualized.

For a detailed explanation on the meaning of the fields of the action goal see the definition of the action messages.