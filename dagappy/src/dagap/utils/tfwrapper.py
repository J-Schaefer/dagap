from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, Transform


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


def quaternion_to_list(quaternion: Quaternion):
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]


def pose_to_list(pose: Pose):
    return [point_to_list(pose.position), quaternion_to_list(pose.orientation)]


def vector3_to_list(vector: Vector3):
    return [vector.x, vector.y, vector.z]


def transform_to_list(transform: Transform):
    return [vector3_to_list(transform.translation), quaternion_to_list(transform.rotation)]
