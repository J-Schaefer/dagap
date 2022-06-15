import rospy

from dagap_msgs.srv import GetGraspPose
from dagap.sample import SampleGrasp
from py_trees import Sequence, Selector, BehaviourTree, Blackboard
from dagap.tree.mesher import Mesher
from dagap.tree.semantic_module import SemanticModule

class DAGAP:
    def __init__(self):
        self.service = rospy.Service('srv_dagap', GetGraspPose, self.cb_service)
        # sim = SampleGrasp('wsg_50', 'cucumber')
        # sim.load_gripper()
        pass

    def cb_service(self, req):
        pass

# TODO: grow tree

    def grow_tree(self):
        rospy.loginfo("Growing tree.")
        preparation = Sequence()
        preparation.add_child(Mesher("Prepare meshes"))  # add Mesher
        preparation.add_child(SemanticModule("Parse command"))

    def run(self):
        sleeper = rospy.Rate(10)  # rate in Hz
        rospy.loginfo('DAGAP is ready')
        while not rospy.is_shutdown():
            try:
                # self.tick()
                sleeper.sleep()
            except KeyboardInterrupt:
                break
