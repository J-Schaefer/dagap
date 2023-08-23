import rospy
from py_trees import Behaviour
import tf
from geometry_msgs.msg import TransformStamped, PoseStamped
from numpy.linalg import norm
import dagap.utils.tfwrapper as dagap_tf


class GraspPlanner(Behaviour):
    def __init__(self):
        rospy.loginfo("Starting grasp planner")
        self.tfl = tf.TransformListener()
        self.tfb = tf.TransformBroadcaster()
        dagap_tf.init()


    def decide(self, object, action, robot, opm_action: bool, reference_frame: str = u"", object_frame: str = None):
        """

        :param object: list of frame names
        :param action: string
        :param robot: Robot Object
        :param opm_action: Boolean if action comes from OPM
        :param reference_frame
        :param object_frame
        :return:
        """
        if action == u"one hand task":
            # start to perform one-handed task
            # check distance to estimate which hand to use

            if opm_action:
                task_object = [object[0]]
            else:
                task_object = object

            try:
                if object_frame:
                    task_object = [object_frame]
                else:
                    rospy.logwarn("Couldn't find object frame.")
            except TypeError:
                rospy.logwarn("Types don't seem to fit.")


            try:
                if len(task_object) == 1:
                    # start to perform one handed task
                    # check distance to estimate which hand to use
                    current_frame = task_object[0]
                    rospy.loginfo("Calculating distance to frame " + current_frame)  # TODO: add prints for more objects
                    pose_list = []
                    distance = []

                    # Add tf_root for simulated robot in bullet world
                    if robot.tf_root:
                        robot.gripper_list = ['{0}{1}'.format(robot.tf_root, gripper) for gripper in robot.gripper_list]

                    for gripper in robot.gripper_list:
                        # FIXME: get tf prefix and concatenate, right now frame cannot be found
                        transform: TransformStamped = dagap_tf.lookup_transform(gripper, current_frame)
                        trans = transform.transform.translation
                        rot = transform.transform.rotation
                        pose_list.append([trans, rot])
                        distance.append(norm(dagap_tf.point_to_list(trans)))  # add distance of current gripper to object to list

                    min_index = distance.index(min(distance))
                    winner = robot.gripper_list[min_index]  # get gripper with smallest distance
                    pose_winner = pose_list[min_index]  # get pose with smallest distance
                    rospy.loginfo(rospy.get_name(), "Found closest gripper: " + winner)
                elif len(task_object) == 2:
                    # More than one object with
                    rospy.logwarn("Not implemented yet. Returning empty.")
                    transform: TransformStamped = TransformStamped()
                    return transform

                else:
                    rospy.logwarn("Too many objects. Returning empty.")
                    transform: TransformStamped = TransformStamped()
                    return transform
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Could not find transform.")

            if opm_action:
                return [object_frame, winner]
            else:
                # print(self.manipulation_cases.get(sentence, "Not found"))
                GraspPose = TransformStamped()
                GraspPose.header.frame_id = winner
                GraspPose.transform = dagap_tf.list_to_transform(pose_winner[0], pose_winner[1])

                return [GraspPose]



        elif action == u"one hand support":
            pass

        elif action == u"fixed offset":
            pass
