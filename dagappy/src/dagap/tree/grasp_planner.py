import rospy
from py_trees import Behaviour
import tf
from geometry_msgs.msg import TransformStamped, PoseStamped
from numpy.linalg import norm
import dagap.utils.tfwrapper as dagap_tf


class GraspPlanner(Behaviour):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        rospy.loginfo("Starting grasp planner")
        self.tfl = tf.TransformListener()
        self.tfb = tf.TransformBroadcaster()
        dagap_tf.init()


    def decide(self, grasping_object, action, robot, opm_action: bool, reference_frame: str = u"", object_frame: str = None):
        """

        :param grasping_object: list of frame names
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
                task_object = [grasping_object[0]]
            else:
                task_object = grasping_object

            try:
                if object_frame:
                    task_object = [object_frame]
                else:
                    rospy.logwarn("Couldn't find object frame.")
            except TypeError:
                rospy.logwarn("Types don't seem to fit.")


            try:
                if len(task_object) == 1:
                    # start to perform one-handed task
                    # check distance to estimate which hand to use
                    current_frame = task_object[0]
                    pose_list = []
                    distance = []
                    gripper_list = []
                    winner: str = ""

                    # Add tf_root for simulated robot in bullet world
                    # use local gripper list and leave robot.gripper_list intact
                    if robot.tf_root:
                        gripper_list = ['{0}/{1}'.format(robot.tf_root, gripper) for gripper in robot.gripper_list]

                    for gripper in gripper_list:  # use local gripper list
                        # FIXME: get tf prefix and concatenate, right now frame cannot be found
                        rospy.loginfo("Calculating distance from {} to {}".format(gripper,
                                                                                  current_frame))
                        transform: TransformStamped = dagap_tf.lookup_transform(gripper, current_frame)
                        rospy.loginfo("[{}]: Found transform".format(rospy.get_name()))
                        trans = transform.transform.translation
                        rot = transform.transform.rotation
                        pose_list.append([trans, rot])
                        distance.append(
                            norm(dagap_tf.point_to_list(trans)))  # add distance of current gripper to object to list

                    min_index = distance.index(min(distance))
                    winner = gripper_list[min_index]  # get gripper with the smallest distance
                    pose_winner = pose_list[min_index]  # get pose with the smallest distance
                    rospy.loginfo("[{}]: Found closest gripper: {}".format(rospy.get_name(), winner))
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
                return [object_frame, winner]  # TODO: in case of an error variable winner is empty
            else:
                # print(self.manipulation_cases.get(sentence, "Not found"))
                GraspPose = TransformStamped()
                GraspPose.header.frame_id = winner
                GraspPose.transform.translation = pose_winner[0]
                GraspPose.transform.rotation = pose_winner[1]

                return [GraspPose]

        elif action == u"one hand support":
            pass

        elif action == u"fixed offset":
            pass
