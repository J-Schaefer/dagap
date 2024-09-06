# README for DAGAP

## Installation

### Install dependencies

First install the run dependencies.

```bash
mkdir -p dagap_ws/src  # do this only when you want to create a new workspace
cd dagap_ws/src  # replace with your workspace name
git clone https://github.com/code-iai/omni_pose_follower.git
git clone https://github.com/code-iai/iai_naive_kinematics_sim.git
git clone https://github.com/code-iai/iai_pr2.git
sudo apt install -y ros-noetic-py-trees
catkin build
```

Then install some Python dependencies:

```bash
python3 -m pip install pybullet
python3 -m pip install nltk
```

Install Giskard following the instructions on the [Giskard Github page](https://github.com/SemRoCo/giskardpy/wiki).



### Install DAGAP

Clone the repository into your workspace and compile the workspace.

```bash
cd dagap_ws/src  # replace with your workspace name
git clone git@github.com:J-Schaefer/dagap.git
catkin build
source ../devel/setup.bash
```

## Run the ROS Service

In order to have a tf tree, please launch a robot simulation or the real robot. For example:

```bash
roslaunch iai_pr2_sim ros_control_sim_with_base.launch
```

or

```bash
roslaunch giskardpy giskardpy_pr2_standalone.launch
```

If you want to use the kitchen or the objects are referenced to the kitchen, please also launch the kitchen bringup:

```bash
roslaunch iai_kitchen kitchen_obj_bringup.launch
```

Then start the dagap launch file:

```bash
roslaunch dagappy dagap.launch
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
