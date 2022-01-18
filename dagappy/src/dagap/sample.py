# Trial and Error
# Call a sim (PyBullet) to sample grasp

import pybullet
import pybullet_data
import os
import rospkg
import rospy


class SampleGrasp():
    def __init__(self, gripper_name, object_name):
        pybullet.connect(pybullet.GUI)
        pybullet.resetSimulation()
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        rospack = rospkg.RosPack()
        self.gripper_path = rospack.get_path('iai_wsg_50_description')

    def load_gripper(self):
        urdf_path = self.gripper_path + '/urdf/wsg_50.urdf.xacro'
        rospy.loginfo("Loading gripper with path " + urdf_path)
        pybullet.loadURDF('/home/jesch/ros_ws/iai_robots_ws/src/iai_pr2/iai_pr2_description/robots/pr2_with_odom_joints.xacro')
        # pybullet.loadURDF(urdf_path)
