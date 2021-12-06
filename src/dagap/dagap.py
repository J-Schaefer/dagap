import rospy


class DAGAP:
    def __init__(self):
        self.service = rospy.ServiceProxy('srv_dagap')
        pass

    def cb_service(self, req):
        pass
