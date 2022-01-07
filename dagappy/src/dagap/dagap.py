import rospy

from dagap_msgs import GetGraspPose

class DAGAP:
    def __init__(self):
        self.service = rospy.Service('srv_dagap', GetGraspPose, self.cb_service)
        pass

    def cb_service(self, req):
        pass
