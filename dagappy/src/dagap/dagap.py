import rospy

from dagap_msgs.srv import GetGraspPose
from dagap.sample import SampleGrasp

class DAGAP:
    def __init__(self):
        self.service = rospy.Service('srv_dagap', GetGraspPose, self.cb_service)
        sim = SampleGrasp('wsg_50', 'cucumber')
        sim.load_gripper()
        pass

    def cb_service(self, req):
        pass

    # TODO: grow tree
