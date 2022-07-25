import rospy

from dagap_msgs.srv import GetGraspPose
from dagap.sample import SampleGrasp
from py_trees import Sequence, Selector, BehaviourTree, Blackboard
from dagap.tree.mesher import Mesher
from dagap.tree.semantic_module import SemanticModule
from dagap.tree.grasp_planner import GraspPlanner


class DAGAP:
    manipulation_cases = {1: u"one hand seeking", 2: u"one hand fixed", 3: u"fixed offset", 4: u"self-handover"}

    def __init__(self):
        self.service = rospy.Service('srv_dagap', GetGraspPose, self.cb_service)
        self.semantic_module = SemanticModule('Semantic Module')
        # self.tree = BehaviourTree(self.grow_dagap())
        # sim = SampleGrasp('wsg_50', 'cucumber')
        # sim.load_gripper()
        pass

    def cb_service(self, req):
        print("Received request.")
        print(req.description)
        sentence = self.semantic_module.define_action(req.description.data)
        print(self.manipulation_cases.get(sentence, "Not found"))
        res = "Bla"
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
        preparation.add_child(SemanticModule("Parse command"))
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
