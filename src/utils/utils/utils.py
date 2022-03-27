from geometry_msgs.msg import Pose, Twist, TransformStamped
import yaml

def yaml2Pose(msg):
    d = yaml.safe_load(msg)

    ret = Pose()

    ret.position.x = d['position']['x']
    ret.position.y = d['position']['y']
    ret.position.z = d['position']['z']

    ret.orientation.x = d['orientation']['x']
    ret.orientation.y = d['orientation']['y']
    ret.orientation.z = d['orientation']['z']
    ret.orientation.w = d['orientation']['w']

    return ret

def yaml2Twist(msg):
    d = yaml.safe_load(msg)

    ret = Twist()

    ret.linear.x = d['linear']['x']
    ret.linear.y = d['linear']['y']
    ret.linear.z = d['linear']['z']

    ret.angular.x = d['angular']['x']
    ret.angular.y = d['angular']['y']
    ret.angular.z = d['angular']['z']
    ret.angular.w = d['angular']['w']

    return ret
