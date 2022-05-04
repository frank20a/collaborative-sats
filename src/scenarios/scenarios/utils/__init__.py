from geometry_msgs.msg import Pose, TransformStamped

def tf2pose(x: TransformStamped) -> Pose:
    pose = Pose()
    pose.position.x = x.transform.translation.x
    pose.position.y = x.transform.translation.y
    pose.position.z = x.transform.translation.z
    pose.orientation.x = x.transform.rotation.x
    pose.orientation.y = x.transform.rotation.y
    pose.orientation.z = x.transform.rotation.z
    pose.orientation.w = x.transform.rotation.w
    return pose