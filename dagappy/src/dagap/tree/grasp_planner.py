import rospy
from py_trees import Behaviour


class GraspPlanner(Behaviour):
    def __init__(self):
        rospy.loginfo("Starting grasp planner")
