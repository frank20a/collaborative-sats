from geometry_msgs.msg import Pose, TransformStamped
from ..tf_utils import quaternion_multiply, vector_rotate_quaternion, quaternion_inverse
import numpy as np

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

def do_transform_pose(pose: Pose, tf: TransformStamped) -> Pose:
    res = Pose()

    p = vector_rotate_quaternion(
        [pose.position.x + tf.transform.translation.x, pose.position.y + tf.transform.translation.y, pose.position.z + tf.transform.translation.z],
        [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w],
    )
    res.position.x = float(p[0])
    res.position.y = float(p[1])
    res.position.z = float(p[2])

    q = np.array(quaternion_multiply(
        [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w],
        quaternion_inverse([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    ))
    q = q / quaternion_norm(q)


    res.orientation.x = q[0]
    res.orientation.y = q[1]
    res.orientation.z = q[2]
    res.orientation.w = q[3]

    return res

def quaternion_norm(q: np.ndarray) -> float:
    return np.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)