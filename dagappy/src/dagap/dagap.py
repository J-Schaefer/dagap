import rospy

from dagap_msgs import GetGraspPose

class DAGAP:
    def __init__(self):
        self.service = rospy.ServiceProxy('srv_dagap', GetGraspPose)
        pass

    def cb_service(self, req):
        pass
