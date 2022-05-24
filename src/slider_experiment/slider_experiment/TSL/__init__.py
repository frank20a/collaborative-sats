from parameters import *
from ament_index_python import get_package_share_directory
import sys, os

sys.path.insert(1, os.path.join(get_package_share_directory('slider_experiment'), 'python_build/tsl_optimizer'))
import tsl_optimizer as optimizer


solver = optimizer.solver()


def force2thrust(Fx, Fy, tau):
    """
    Convert force/torque vectors to thrust
    """