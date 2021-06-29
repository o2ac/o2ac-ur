# Flexbe Instructions

## Running an existing example

1. Launch flexbe GUI editor
    ```
    roslaunch flexbe_app flexbe_full.launch
    ```
2. Load the `bearing taskboard` example using the `load behavior` bottom at the top
3. Click `run time control`
4. Click `start execution`


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
