import rospy
from py_trees import Behaviour
import tf
import geometry_msgs
from numpy.linalg import norm

class GraspPlanner(Behaviour):
    def __init__(self):
        rospy.loginfo("Starting grasp planner")
        self.tfm = tf.TransformListener()

    def decide(self, objects, action, robot):
        if action == u"one hand task":
            # start to perform one handed task
            # check distance to estimate which hand to use
            try:
                task_object = objects[0]
                rospy.loginfo("Calculating distance to " + task_object)  # TODO: add prints for more objects
                pose_list = []
                distance = []
                for gripper in robot.gripper_list:
                    (trans, rot) = self.tfm.lookupTransform(gripper, task_object, rospy.Time(0))
                    pose_list.append([trans, rot])
                    distance.append(norm(trans))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Could not find transform.")
            # print(self.manipulation_cases.get(sentence, "Not found"))
            Pose1 = geometry_msgs.msg.PoseStamped()
            Pose2 = geometry_msgs.msg.PoseStamped()

        return [Pose1, Pose2]
