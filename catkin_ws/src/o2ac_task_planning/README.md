# o2ac_task_planning

NOTE: This package was not actively used during the WRS2020, but is a prototype extending MoveIt Task Constructor with symbolic task planning.

This package starts multiple action servers for different tasks, such as `Pick Object`, `Place Object` etc. The action servers do the task planning when a request arrives and they return the best solution if it is found, which contains the trajectories for executing the plan.

## QUICKSTART

In separate terminals:

```bash
roslaunch o2ac_moveit_config demo.launch
# Publish the parameters. TODO(karolyartur)
rosrun o2ac_task_planning_core mtc_modules_server 
rosrun downward fast-downward-server.py
```

Launch symbolic planning with `rosrun o2ac_task_planning_pddl_converter fast_downward_client.py`. Launch MTC planning with what? **TODO(karolyartur)**

## Running mtc_modules_server

### Requirements

The node requires certain parameters to be set on the ROS parameter server for its initialization. **These are used as default values** for the tasks' parameters (the parameters that most likely do not change for each call to the action servers, such as the name of the robots, directions for lifting, or placing objects, the name of the support surfaces ...)

( `rosrun o2ac_routines assembly.py` )

An example for setting these parameters can be found [here](https://gitlab.com/o2ac/o2ac-ur/-/blob/integrate-mtc-in-o2ac-routines/catkin_ws/src/o2ac_routines/src/o2ac_routines/helpers.py#L29).

Before starting the node make sure that these parameters are available on the parameter server:

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

The node relies on a stage that looks for and loads the possible grasps from the ROS parameter server instead of the [GenerateGraspPose](https://gitlab.com/o2ac/moveit-task-constructor/-/blob/master/core/src/stages/generate_grasp_pose.cpp) stage in MTC.

The grasps are expected to be in the format, in which [o2ac_assembly_database](https://gitlab.com/o2ac/o2ac-ur/-/tree/integrate-mtc-in-o2ac-routines/catkin_ws/src/o2ac_assembly_database) package uploads the grasps to the parameter server ([here](https://gitlab.com/o2ac/o2ac-ur/-/blob/integrate-mtc-in-o2ac-routines/catkin_ws/src/o2ac_assembly_database/src/o2ac_assembly_database/assy.py#L61)).

However, the grasps can be set directly (the use of o2ac_assembly_handler is not necessary, it is just convenient fo the objects that are already part of that package). For new objects (like the tools for example), the grasps can be defined directly as well, like [this](https://gitlab.com/o2ac/o2ac-ur/-/blob/integrate-mtc-in-o2ac-routines/catkin_ws/src/o2ac_routines/src/o2ac_routines/base.py#L696).

The grasps on the parameter server are expected to be in the following format:

```
/assembly_level_namespace/object_level_namespace/grasp_level_namespace/position  # A list of three values ([x,y,z])
/assembly_level_namespace/object_level_namespace/grasp_level_namespace/orientation  # A list of four values (quaternion, [x,y,z,w])
```

Getting the `/assembly_level_namespace/object_level_namespace/grasp_level_namespace` parameter enables the retrieval of a single grasp of a single object. The returned value is a dictionary of the form: `{"position":[], "orientation":[]}`

To get all the grasps of a single object the parent parameter can be retrieved, such as `/assembly_level_namespace/object_level_namespace`. This returns a dictionary of dictionaries that contains all the grasps of a single object, like: `{"grasp_1":{"position":[], "orientation":[]}, "grasp_2":{"position":[], "orientation":[]}, ...}`

Retrieving `/assembly_level_namespace` parameter only, returns all of the grasps of all of the objects in this collection.

The `mtc_modules/grasp_parameter_location` parameter refers to the `assembly_level_namespace` and is used to retrieve the correct grasps. The `object_name` field of the action messages refers to the `object_level_namespace`. Please note, that the planners expect the grasps to be loaded to the parameter server and the planning will not be successful if the grasps are not available.

### EEF names and frames

The naming of the eef (defined in the [srdf](https://gitlab.com/o2ac/o2ac-ur/-/blob/integrate-mtc-in-o2ac-routines/catkin_ws/src/o2ac_moveit_config/config/o2ac_base_scene.srdf#L219)) is expected to follow the pattern:

`eef_name` = `arm_group_name` + `_tip`,

where `arm_group_name` is the name of one of the planning groups of the robot arms (given by the `mtc_modules/arm_group_names` required parameter).

The naming of the eef frames are expected to follow the pattern:

`eef_frame_name` = `hand_group_name` + `_tip_link`,

where `hand_group_name` is the name of one of the planning groups of the grippers (given by the `mtc_modules/hand_group_names` required parameter).

### MoveIt

For the planning to work, MoveIt has to be running. Make sure to start MoveIt before running the node:

```bash
roslaunch o2ac_moveit_config demo.launch
```

### Running the node

**Before running the node make sure that the required parameters (initial parameters and grasps) are loaded to the parameter server, MoveIt is running and the required collision objects are in the scene.**

**TODO(karolyartur): Point to an explanation on how that's done, be specific with instructions like "the required X" (the people who need to read instructions generally don't know what is required)**

To run the node type:

```bash
rosrun o2ac_task_planning_core mtc_modules_server 
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
 - `/wrs_subtask_b_planning` (picking and placing the L panel onto the base plate and fixing it with the screws)

To start planning, make requests from an action client to use the servers. An easy way to test them is with the `actionlib` package.

To test the action server for pick planning, type:

 ```bash
 rosrun actionlib axclient.py /pick_planning
 ```
This opens a new window where the action goal can be set and the feedback/result can be inspected, and it looks like this:

<div align="center">
    <img src="/uploads/ed309ffb1982093b0bf14cde9baa6108/Screenshot_2020-08-19_17_18_33.png" width="30%" height="30%">
</div>

After sending the goal, the planning should start. This can be seen in the terminal in which `mtc_modules_server` is running, or with RViz.

To visualize the task planning with RViz add the `Motion Planning Tasks` display.

<div align="center">
    <img src="/uploads/82c609bf04167baa6ff53d4b9d0d8587/Screenshot_2020-08-19_17_43_47.png" width="60%" height="60%">
</div>

In the `Motion Planning Tasks` panel, under `Task Tree` the hierarchical representation of the task should be visible if the planning has started. Selecting one of the stages in the task should reveal the solutions of the selected stage on the right side of the `Motion Planning Tasks` panel. By clicking on one of the solutions, the motion plan can be visualized.

<div align="center">
    <img src="/uploads/c2ea23252b9831a74a0b6da6cf2d18af/Screenshot_2020-08-19_17_43_48.png" width="60%" height="60%">
</div>

For a detailed explanation on the meaning of the fields of the action goal see the definition of the [action messages](https://gitlab.com/o2ac/o2ac-ur/-/tree/integrate-mtc-in-o2ac-routines/catkin_ws/src/o2ac_task_planning/msgs/action).


For an example use case, first call:

```bash
roslaunch o2ac_moveit_config demo.launch 
```

This starts moveit and loads the o2ac demo scene (to try the same thing with the real robots, use o2ac_moveit_planning_execution.launch instead of demo.launch).

Next start [assembly.py](https://gitlab.com/o2ac/o2ac-ur/-/blob/integrate-mtc-in-o2ac-routines/catkin_ws/src/o2ac_routines/scripts/assembly.py) from `o2ac_routines`in a separate terminal:

```bash
rosrun o2ac_routines assembly.py 
```
and type `68`. This spawns the collision objects in the scene and [sets the required parameters](https://gitlab.com/o2ac/o2ac-ur/-/blob/integrate-mtc-in-o2ac-routines/catkin_ws/src/o2ac_routines/src/o2ac_routines/helpers.py#L29).

After this start the action servers:

```bash
rosrun o2ac_task_planning_core mtc_modules_server
```

If the server is running, use the terminal of `assembly.py` and type `75` to start the subassembly planning.

You can check that the planning is running in the terminal of `mtc_modules_server` and in RViz as well.

If the planning finished successfully, the **best solution** (solution with the lowest cost) gets written into a file.

To execute the solution start `o2ac_skill_server`:

```bash
roslaunch o2ac_skills o2ac_skill_server.launch
```

and type `80` in the terminal of `assembly.py`

## Symbolic planning (how to use PDDL)

We use PDDL ([Planning Domain Definition Language](https://en.wikipedia.org/wiki/Planning_Domain_Definition_Language)) to define symbolic planning problems and domains. Sample definitions are in o2ac_assembly_handler/pddl. Use the osx-assembly-v2.pddl domain with the problem-v3.pddl problem. For how to use the PDDL planner see the [README](catkin_ws/src/o2ac_assembly_handler/pddl/README.md) in o2ac_assembly_handler/pddl.

If a solution was found, the result (sas_plan.1) will appear in o2ac_assembly_handler/pddl. Copy this file to o2ac_task_planning/pddl_converter/plans.


After that, launch the o2ac demo scene

```bash
roslaunch o2ac_moveit_config demo.launch
```

In a separate terminal run

```bash
rosrun o2ac_routines assembly.py 
```
and type `68`. This spawns the collision objects in the scene and [sets the required parameters](https://gitlab.com/o2ac/o2ac-ur/-/blob/integrate-mtc-in-o2ac-routines/catkin_ws/src/o2ac_routines/src/o2ac_routines/helpers.py#L29).

After this, start the action servers in a new terminal:

```bash
rosrun o2ac_task_planning_core mtc_modules_server
```

If the action servers are running, open a new terminal and call:

```bash
rosrun o2ac_task_planning_pddl_converter pddl_converter.py 
```

This should read the PDDL `trace` from o2ac_task_planning/pddl_converter/plans and convert it to an MTC task, and start the planning of the task.

The parsing of the PDDL `trace` is performed line-by-line, excluding commented-out lines (starting with ";").

The structure of each line follows the pattern:

```
(action_name **parameters**)
```

The parameters are the names of the robots, objects, tools etc., that take part in performing the action, separated by a whitespace.

The definition of the actions provided by the assembly domain can be found in the [osx-assembly-v2.pddl file](catkin_ws/src/o2ac_assembly_handler/pddl/osx-assembly-v2.pddl).

The format of the pick action for example:
```
(pick robot object helper_robot),
```
where `robot` is the name of the robot holding the object at the end of the pick action,

`object` is the name of the object,

`helper_robot` is the name of the other robot that can pick the object and hand it over to `robot` if a regrasp is needed


The planning process and the results of the planning can be inspected in RViz.
