import rospy
from py_trees import Behaviour
import tf
import geometry_msgs

class GraspPlanner(Behaviour):
    def __init__(self):
        rospy.loginfo("Starting grasp planner")
        self.tfm = tf.TransformListener()

    def decide(self, objects, action):
        # if
        try:
            (trans, rot) = self.tfm.lookupTransform('/r_gripper_tool_frame', '/l_gripper_tool_frame', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Could not find transform.")
        # print(self.manipulation_cases.get(sentence, "Not found"))
        Pose1 = geometry_msgs.msg.PoseStamped()
        Pose2 = geometry_msgs.msg.PoseStamped()

        return [Pose1, Pose2]
