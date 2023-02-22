from geometry_msgs.msg import Point, Pose, Quaternion


def list_to_point(point: list):
    return Point(point[0], point[1], point[2])


def list_to_quaternion(point: list):
    return Quaternion(point[0], point[1], point[2], point[3])


def list_to_pose(position: list, orientation: list):
    return Pose(list_to_point(position), list_to_quaternion(orientation))


def point_to_list(point: Point):
    return [point.x, point.y, point.z]


def quaternion_to_list(quaternion: Quaternion):
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]


def pose_to_list(pose: Pose):
    return [point_to_list(pose.position), [quaternion_to_list(pose.orientation)]]
