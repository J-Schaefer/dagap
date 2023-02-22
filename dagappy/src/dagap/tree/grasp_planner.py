import rospy
from py_trees import Behaviour
import tf
import geometry_msgs
from numpy.linalg import norm


class GraspPlanner(Behaviour):
    def __init__(self):
        rospy.loginfo("Starting grasp planner")
        self.tfl = tf.TransformListener()
        self.tfb = tf.TransformBroadcaster()

    def decide(self, object, action, robot, opm_action: bool, reference_frame: str = u""):
        """

        :param object: list of frame names
        :param action: string
        :param robot: Robot Object
        :param opm_action: Boolean if action comes from OPM
        :param reference_frame
        :return:
        """
        if action == u"one hand task":
            if len(object) == 1:
                # start to perform one handed task
                # check distance to estimate which hand to use

                if opm_action:

                    self.tfb.sendTransform(object[1][0],
                                           object[1][1],
                                           rospy.Time.now(),
                                           object[0],
                                           reference_frame)
                    task_object = object[0]
                else:
                    task_object = object[0]

                try:
                    rospy.loginfo("Calculating distance to " + task_object)  # TODO: add prints for more objects
                    pose_list = []
                    distance = []
                    for gripper in robot.gripper_list:
                        # FIXME: get tf prefix and concatenate, right now frame cannot be found
                        (trans, rot) = self.tfl.lookupTransform(gripper, task_object, rospy.Time(0))
                        pose_list.append([trans, rot])
                        distance.append(norm(trans))
                    winner = robot.gripper_list[distance.index(min(distance))]
                    rospy.loginfo("Found closest gripper: " + winner)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logerr("Could not find transform.")

                if opm_action:
                    return [task_object, winner]
                else:
                    # print(self.manipulation_cases.get(sentence, "Not found"))
                    GraspPose = geometry_msgs.msg.TransformStamped()
                    GraspPose.header.frame_id = winner
                    GraspPose.transform.translation.x = pose_list[distance.index(min(distance))][0][0]
                    GraspPose.transform.translation.y = pose_list[distance.index(min(distance))][0][1]
                    GraspPose.transform.translation.z = pose_list[distance.index(min(distance))][0][2]
                    GraspPose.transform.rotation.x = pose_list[distance.index(min(distance))][1][0]
                    GraspPose.transform.rotation.y = pose_list[distance.index(min(distance))][1][1]
                    GraspPose.transform.rotation.z = pose_list[distance.index(min(distance))][1][2]
                    GraspPose.transform.rotation.w = pose_list[distance.index(min(distance))][1][3]

                    return [GraspPose]

            elif len(object) == 2:
                # More than one object with
                rospy.logwarn("Not implemented yet. Returning empty.")
                Pose = geometry_msgs.msg.PoseStamped()
                return Pose

            else:
                rospy.logwarn("Too many objects. Returning empty.")
                Pose = geometry_msgs.msg.PoseStamped()
                return Pose

        elif action == u"one hand support":
            pass

        elif action == u"fixed offset":
            pass

        elif action == u"self-handover":
            pass
