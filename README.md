# README for DAGAP

## Installation

### Install dependencies

Install Giskard following the instructions on the [Giskard Github page](https://github.com/SemRoCo/giskardpy/wiki).

### Install DAGAP

Clone the repository into your workspace and compile the workspace.

```bash
mkdir -p dagap_ws/src
cd dagap_ws/src
git clone git@github.com:J-Schaefer/dagap.git
catkin build
source ../devel/setup.bash
```

## Run the ROS Service

```bash
roslaunch dagappy dagap.launch
```

If the objects are referenced to the kitchen, please also launch the kitchen:

```bash
roslaunch iai_kitchen kitchen_obj_bringup.launch
```

## Call the Service

The launch file launches three services that allow the user to call DAGAP, OPM and both together:

* dagap_query
* opm_query
* dagap_opm_query

### DAGAP Query

Receives the task description in form of a short natural language sentence and the tf frames of the object(s).
Returns the object poses in the gripper tool frame as reference.\
srv type: `GetGraspPose`

Input\
`std_msgs/String description`\
`std_msgs/String[] object_frames`\
Output\
`geometry_msgs/TransformStamped[] grasp_pose`


### OPM Query

Receives an unsorted list of objects and returns the next object to be grasped.\
srv type: `GetOPMSortedList`

Input\
`OPMObjectQuery[] object_list`, the list needs to have one object called "robot" with the current position.\
Output\
`string next_object`


### Combined Query

Received an unsorted list of objects and returns the next object and gripper to grasp with.\
srv type: `GetOPMSortedList`

Input\
`string reference_frame`
`OPMObjectQuery[] object_list`, the list needs to have one object called "robot" with the current position.\
Output\
`string next_object`\
`string hand`

## Authors

Jeroen Schäfer - [jeroen.schaefer@outlook.de](mailto:jeroen.schaefer@outlook.de)\
OPM Part: Petra Wenzl - [pwenzl@uni-bremen.de](mailto:pwenzl@uni-bremen.de)
