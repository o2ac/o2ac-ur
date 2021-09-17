# Flexbe Instructions

## Running an existing example

1. Launch the python skill server
    ```
    rosrun o2ac_routines skill_server.py
    ```
2. Launch flexbe GUI editor
    ```
    roslaunch flexbe_app flexbe_full.launch
    ```
3. Load the `bearing taskboard` example using the `load behavior` bottom at the top
4. Click `run time control`
5. Click `start execution`

# From Task skeleton to Flexbe behavior
A PDDL task skeleton demo can be found in `o2ac_flexbe_behaviors/config/test_pddl` with the following content:
```
pick b_bot panel_bearing
handover b_bot a_bot panel_bearing
orient a_bot assembly panel_bearing
fasten b_bot panel_bearing
```
To create a Flexbe behavior from this PDDL example, open flexbe and use the button `Behavior Dashboard > Load Task Skeleton` from the toolbar above.
The new behavior will be populated, saving this new behavior is required before execution.

The demo logic for this functionality can be found here:

`underlay_ws/src/third_party/flexbe_app/src/ui/ui_menu.js > this.loadTaskClicked()`

TODO:
- Add a selector for input PDDL file
- Add more action states

## Creating a new behavior
http://wiki.ros.org/flexbe/Tutorials/Creating%20a%20New%20Behavior


# Skill dataset
The idea is to have a dataset of general skills that can be easily assembled together to create a behavior such as assembly of a product.
Keep the skills general and independent of each other as much as possible.

Inputs are based on what would be obtained from the symbolic planner, e.g. robot A picks object B


### Pick
Grasp an object, look for it, compute grasp pose, execute motion. Optionally a target grasp can be specified.

- Inputs:
    - Object Name # use to simplify parametrization, based on this object name, we load any specific parameters from the assembly database
    - Robot Name
    - Target Grasp Pose (optional) # would trigger an orient skill if needed
- Result:
    - Success (Bool)
 

### Orient
Update robot's grasp pose of target object by in-hand manipulation, extrinsic manipulation, hand-over, etc.

- Inputs:
    - Object Name
    - Robot Name
    - Target Grasp Pose 
- Result:
    - Success (Bool)
 
### MoveTo
Move robot to a target pose

- Inputs:
    - Robot Name
    - Target Pose 
    - Pose Type # Target joints values (default) or Target cartesian pose
    - Frame ID (Optional) # Only used for target cartesian poses, default is `world`
    - Target Named Pose (Optional) # if given then, `Target Pose` is ignored
    - Motion Planner (Linear, PTP, OMPL, ...)
- Result:
    - Success

### Insert
Insert object into target position using force control

- Inputs:
    - Robot Name
    - Target Pose # Target cartesian pose
    - Frame ID # of target pose
    - Target Force
    - Insertion Direction
- Result:
    - Success

### PlayBackSequence
Execute a manually defined motion trajectory

- Inputs:
    - Sequence Name
- Result:
    - Success

### Fasten
Equip a tool, a screw and insert it into a target hole

- Inputs:
    - Robot Name
    - Screw size
    - Tool Name (Optional) # When multiple tools are available for the same `screw size`
    - Target Hole Pose
    - Frame ID
- Result:
    - Success

### Equip/Un-equip Tool

- Inputs:
    - Robot Name
    - Tool Name
    - Operation # Equip/Un-equip/Realign
- Result:
    - Success

### Align Bearing Holes
Very specific task for the Bearing

- Input:
    - Task Name
- Result:
    - Success
