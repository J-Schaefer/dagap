import geometry_msgs.msg
import rospy

import dagap_msgs.srv
from dagap_msgs.srv import GetGraspPose
# from dagap.sample import SampleGrasp
from py_trees import Sequence, Selector, BehaviourTree, Blackboard
from dagap.tree.mesher import Mesher
from dagap.tree.nl_module import NLModule
from dagap.tree.grasp_planner import GraspPlanner
from dagap.tree.robot import Robot
import tf


class DAGAP:
    manipulation_cases = {1: u"one hand seeking", 2: u"one hand fixed", 3: u"fixed offset", 4: u"self-handover"}

    def __init__(self):
        self.service = rospy.Service('dagap_query', GetGraspPose, self.cb_service)
        self.nl_module = NLModule('Natural Language Module')
        self.robot = Robot()
        self.grasp_planner = GraspPlanner()

        # self.tfm = tf.TransformListener()
        # self.tree = BehaviourTree(self.grow_dagap())
        # sim = SampleGrasp('wsg_50', 'cucumber')
        # sim.load_gripper()
        pass

    def cb_service(self, req):
        rospy.loginfo("Received request.")
        rospy.loginfo("Request description: " + str(req.description))
        rospy.loginfo(req.object_frames)

        action = self.nl_module.define_action(req.description.data)
        if isinstance(action, int):
            rospy.logerr("Error in NL module. Exiting.")
        else:  # no error in NL processing
            poses = self.grasp_planner.decide(objects=req.object_frames, action=action, robot=self.robot)

            res = dagap_msgs.srv.GetGraspPoseResponse(poses)
            return res

    # TODO: grow tree

    def grow_dagap(self):
        rospy.loginfo("Growing tree.")
        root = Sequence("DAGAP")
        root.add_child(self.grow_preparation())
        return root

    def grow_preparation(self):
        preparation = Sequence("Preparation")
        preparation.add_child(Mesher("Prepare meshes"))  # add Mesher
        preparation.add_child(NLModule("Parse command"))
        return preparation

    def run(self):
        sleeper = rospy.Rate(10)  # rate in Hz
        rospy.loginfo('DAGAP is ready')
        while not rospy.is_shutdown():
            try:
                # self.tick()
                sleeper.sleep()
            except KeyboardInterrupt:
                break
