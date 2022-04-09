
import numpy as np
from filterpy.kalman import KalmanFilter


class ConstAccel(KalmanFilter):
    def __init__(self, dt = 1.0/30):
        super().__init__(dim_x=18, dim_z=6)
        
        self.x = np.zeros(18)
        
        # Transition matrix position/orientation
        tmp = np.eye(9, dtype=np.float32)
        tmp[0:6, 3:9] = np.eye(6, dtype=np.float32) * dt
        tmp[0:3, 6:9] = np.eye(3, dtype=np.float32) * dt * dt / 2
        
        # Transition matrix
        self.F = np.concatenate((
            np.concatenate((tmp, np.zeros((9, 9))), axis=1), 
            np.concatenate((np.zeros((9, 9)), tmp), axis=1)), 
            axis=0
        )
        
        # Measurement matrix
        tmp = np.zeros((6, 18), dtype=np.float32)
        tmp[0:3, 0:3] = np.eye(3, dtype=np.float32)
        tmp[3:6, 9:12] = np.eye(3, dtype=np.float32)
        self.H = tmp
        
        # Process/Measurement noise covariance & Error
        # self.processNoiseCov = 1e-5 * np.eye(18)
        # self.measurementNoiseCov = 1e-4 * np.eye(6)
        # self.errorCovPost = 1. * np.eye(18)
        # self.statePost = 1. * np.zeros((18, 1))
        
        # Nose matrices
        self.P *= 1e4      # Covariance matrix
        self.R *= 1e-2     # Measurement noise
        self.Q *= 1e-6     # Process noise


class ConstVel(KalmanFilter):
    def __init__(self, dt = 1.0/30):
        super().__init__(dim_x=12, dim_z=6)
        
        self.x = np.zeros(12)
        
        # Transition matrix position/orientation
        tmp = np.eye(6, dtype=np.float32)
        tmp[0:3, 3:6] = np.eye(3, dtype=np.float32) * dt
        
        # Transition matrix
        self.F = np.concatenate((
            np.concatenate((tmp, np.zeros((6, 6))), axis=1), 
            np.concatenate((np.zeros((6, 6)), tmp), axis=1)), 
            axis=0
        )
        
        # Measurement matrix
        tmp = np.zeros((6, 12), dtype=np.float32)
        tmp[0:3, 0:3] = np.eye(3, dtype=np.float32)
        tmp[3:6, 6:9] = np.eye(3, dtype=np.float32)
        self.H = tmp
        
        # Process/Measurement noise covariance & Error
        # kf.processNoiseCov = 1e-5 * np.eye(18, dtype=np.float32)
        # kf.measurementNoiseCov = 1e-4 * np.eye(6, dtype=np.float32)
        # kf.errorCovPost = np.eye(18, dtype=np.float32)
        
        # Nose matrices
        self.P *= 1e4       # Covariance matrix
        self.R *= 1e-2     # Measurement noise
        self.Q *= 1e-6     # Process noise


class ConstAccelQuat(KalmanFilter):
    def __init__(self, dt = 1.0/30):
        super().__init__(dim_x=18, dim_z=6)
        
        self.x = np.zeros(18)
        
        # Transition matrix position/orientation
        tmp = np.eye(9, dtype=np.float32)
        tmp[0:6, 3:9] = np.eye(6, dtype=np.float32) * dt
        tmp[0:3, 6:9] = np.eye(3, dtype=np.float32) * dt * dt / 2
        
        # Transition matrix
        self.F = np.concatenate((
            np.concatenate((tmp, np.zeros((9, 9))), axis=1), 
            np.concatenate((np.zeros((9, 9)), tmp), axis=1)), 
            axis=0
        )
        
        # Measurement matrix
        tmp = np.zeros((6, 18), dtype=np.float32)
        tmp[0:3, 0:3] = np.eye(3, dtype=np.float32)
        tmp[3:6, 9:12] = np.eye(3, dtype=np.float32)
        self.H = tmp
        
        # Noise matrices
        self.P *= 1e4       # Covariance matrix
        self.Q *= 1e-2      # Process noise
        self.R *= 1e-3      # Measurement noise

filters = {
    'const_accel': ConstAccel,
    'const_vel': ConstVel,
    'const_accel_quat': ConstAccelQuat,
    # 'const_vel_quat': ConstVelQuat,
}