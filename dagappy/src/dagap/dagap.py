import geometry_msgs.msg
import rospy

import dagap_msgs.srv
from dagap_msgs.srv import GetGraspPose, GetOPMSortedList, GetNextOPMObject
# from dagap.sample import SampleGrasp
from py_trees import Sequence, Selector, BehaviourTree, Blackboard
from dagap.tree.mesher import Mesher
from dagap.tree.nl_module import NLModule
from dagap.tree.grasp_planner import GraspPlanner
from dagap.tree.robot import Robot
from dagap.tree.opm.opm import OPM
import tf


class DAGAP:
    manipulation_cases = {1: u"one hand seeking", 2: u"one hand fixed", 3: u"fixed offset", 4: u"self-handover"}

    def __init__(self):
        # Query to DAGAP, req: objects to manipulate, res: hand to manipulate with
        self.service_dagap_query = rospy.Service('dagap_query', GetGraspPose, self.cb_dagap_service)
        # Service to query OPM, req: list of objects, res: sorted list of objects
        self.service_opm_query = rospy.Service('opm_query', GetOPMSortedList, self.cb_opm_service)
        # Query to both, req: list of objects, res: next object with hand
        self.service_common_query = rospy.Service('dagap_opm_query', GetNextOPMObject, self.cb_common_service_query)

        self.nl_module = NLModule('Natural Language Module')
        self.robot = Robot()
        self.grasp_planner = GraspPlanner()
        self.opm = OPM()

        # self.tfm = tf.TransformListener()
        # self.tree = BehaviourTree(self.grow_dagap())
        # sim = SampleGrasp('wsg_50', 'cucumber')
        # sim.load_gripper()

    def cb_dagap_service(self, req):
        rospy.loginfo("Received DAGAP request.")
        rospy.loginfo("Request description: " + str(req.description))
        rospy.loginfo(req.object_frames)

        action = self.nl_module.define_action(req.description.data)
        if isinstance(action, int):
            rospy.logerr("Error in NL module. Exiting.")
        else:  # no error in NL processing
            object_frames = []
            for element in req.object_frames:
                object_frames.append(element.data)

            poses = self.grasp_planner.decide(object=object_frames, action=action, robot=self.robot, opm_action=False)
            res = dagap_msgs.srv.GetGraspPoseResponse(poses)
            return res

    # Query to only query OPM
    def cb_opm_service(self, req):
        # TODO: add service to get sorted list from OPM
        pass

    # Query for next object and how to grasp
    def cb_common_service_query(self, req):
        rospy.loginfo("Received OPM/DAGAP request.")

        lst = req.object_list
        reference_frame = req.reference_frame

        next_object = self.opm.predict_next_action(lst)

        next_grasp = self.grasp_planner.decide(object=next_object,
                                               action=u"one hand task",
                                               robot=self.robot,
                                               opm_action=True,
                                               reference_frame=reference_frame)
        res = dagap_msgs.srv.GetNextOPMObjectResponse(next_grasp[0], next_grasp[1])
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
