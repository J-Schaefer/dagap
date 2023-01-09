import rospy
from py_trees import Behaviour
import tf
import geometry_msgs

class GraspPlanner(Behaviour):
    def __init__(self):
        rospy.loginfo("Starting grasp planner")
        self.tfm = tf.TransformListener()

    def decide(self, objects, action, robot):
        if action == u"one hand task":
            # start to perform one handed task
            # check distance to estimate which hand to use
            try:
                for gripper in robot.gripper_list:
                    (trans, rot) = self.tfm.lookupTransform(gripper, objects, rospy.Time(0))
                    pose_list = []
                    pose_list.append([trans, rot])
                    # self.tfm.
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Could not find transform.")
            # print(self.manipulation_cases.get(sentence, "Not found"))
            Pose1 = geometry_msgs.msg.PoseStamped()
            Pose2 = geometry_msgs.msg.PoseStamped()

        return [Pose1, Pose2]
