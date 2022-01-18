import pybullet
import pybullet_data
import os

pybullet.connect(pybullet.GUI)
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = pybullet.loadURDF("plane.urdf")

os.system("git clone https://github.com/ros-industrial/kuka_experimental.git")
robot = pybullet.loadURDF("pr2_with_odom_joints.urdf",
                          [0, 0, 0], useFixedBase=1)
position, orientation = pybullet.getBasePositionAndOrientation(robot)
pybullet.getNumJoints(robot)
joint_index = 2
joint_info = pybullet.getJointInfo(robot, joint_index)
name, joint_type, lower_limit, upper_limit = \
    joint_info[1], joint_info[2], joint_info[8], joint_info[9]

joint_positions = [j[0] for j in pybullet.getJointStates(robot, range(6))]
world_position, world_orientation = pybullet.getLinkState(robot, 2)[:2]

pybullet.setGravity(0, 0, -9.81)   # everything should fall down
pybullet.setTimeStep(0.0001)       # this slows everything down, but let's be accurate...
pybullet.setRealTimeSimulation(0)  # we want to be faster than real time :)

pybullet.setJointMotorControlArray(
    robot, range(6), pybullet.POSITION_CONTROL,
    targetPositions=[0.1] * 6)

for i in range(10000):
    pybullet.stepSimulation()
    if i % 1000 == 0:
        print("hello")
