import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3, Transform, TransformStamped
from tf2_ros import Buffer, TransformListener, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose

tf_buffer: Buffer = None

def init(tf_buffer_size=15):
    global tf_buffer
    tf_buffer = Buffer(rospy.Duration(tf_buffer_size))
    tf_listener = TransformListener(tf_buffer)


def get_tf_buffer():
    global tf_buffer
    return tf_buffer


def list_to_point(point: list):
    return Point(point[0], point[1], point[2])


def list_to_quaternion(point: list):
    return Quaternion(point[0], point[1], point[2], point[3])


def list_to_pose(position: list, orientation: list):
    return Pose(list_to_point(position), list_to_quaternion(orientation))


def list_to_vector3(message: list):
    return Vector3(message[0], message[1], message[2])


def list_to_transform(translation: list, rotation: list):
    return Transform(list_to_vector3(translation), list_to_quaternion(rotation))


def point_to_list(point: Point):
    return [point.x, point.y, point.z]


def point_to_tuple(point: Point) -> tuple:
    return point.x, point.y, point.z


def quaternion_to_list(quaternion: Quaternion):
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]


def quaternion_to_tuple(quaternion: Quaternion) -> tuple:
    return quaternion.x, quaternion.y, quaternion.z, quaternion.w


def pose_to_list(pose: Pose):
    return [point_to_list(pose.position), quaternion_to_list(pose.orientation)]


def vector3_to_list(vector: Vector3):
    return [vector.x, vector.y, vector.z]


def transform_to_list(transform: Transform):
    return [vector3_to_list(transform.translation), quaternion_to_list(transform.rotation)]


def lookup_transform(target_frame, source_frame, time=0, timeout=5.0) -> TransformStamped:
    local_tf_buffer = get_tf_buffer()
    try:
        return local_tf_buffer.lookup_transform(target_frame=target_frame,
                                                source_frame=source_frame,
                                                time=rospy.Time(time),
                                                timeout=rospy.Duration(timeout))
    except ExtrapolationException:
        return TransformStamped()


def transform_pose(pose, target_frame, source_frame) -> PoseStamped:
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = source_frame
    return do_transform_pose(pose_stamped, lookup_transform(target_frame, source_frame))
