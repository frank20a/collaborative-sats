from geometry_msgs.msg import Pose, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
import numpy as np
from tf_transformations import quaternion_inverse, quaternion_multiply


def get_pose_diff(target: Pose, pose: Pose) -> Pose:
    res = Pose()
    res.position.x = pose.position.x - target.position.x
    res.position.y = pose.position.y - target.position.y
    res.position.z = pose.position.z - target.position.z
    
    q_new = quaternion_multiply(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
        quaternion_inverse([target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w])
    )
    res.orientation.x = q_new[0]
    res.orientation.y = q_new[1]
    res.orientation.z = q_new[2]
    res.orientation.w = q_new[3]

    return res


def odometry2tfstamped(odom: Odometry) -> TransformStamped:
    t = TransformStamped()
    
    t.transform.translation.x = odom.pose.pose.position.x
    t.transform.translation.y = odom.pose.pose.position.y
    t.transform.translation.z = odom.pose.pose.position.z
    t.transform.rotation = odom.pose.pose.orientation
    
    return t


def odometry2pose(odom: Odometry) -> Pose:
    pose = Pose()
    
    pose.position.x = odom.pose.pose.position.x
    pose.position.y = odom.pose.pose.position.y
    pose.position.z = odom.pose.pose.position.z
    pose.orientation = odom.pose.pose.orientation
    
    return pose


def q_mul(q1, q2):
    res = Quaternion()
    
    tmp = quaternion_multiply(
        (q1.x, q1.y, q1.z, q1.w),
        (q2.x, q2.y, q2.z, q2.w)
    )
    res.x = tmp[0]
    res.y = tmp[1]
    res.z = tmp[2]
    res.w = tmp[3]
    
    return res


def vector_rotate_quaternion(v, q):
    res = np.zeros(3)
    
    res = quaternion_multiply(
        (q[0], q[1], q[2], q[3]),
        (v[0], v[1], v[2], 0)
    )
    
    res = quaternion_multiply(
        res,
        quaternion_inverse((q[0], q[1], q[2], q[3]))
    )
    
    return np.array(res[0:3], dtype=np.float32)


def odometry2array(odom: Odometry) -> np.ndarray:
    pose = odometry2pose(odom)
    return np.array([pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
