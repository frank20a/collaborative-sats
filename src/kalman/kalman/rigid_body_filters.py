import cv2 as cv
import numpy as np


def const_accell(dt = 1.0/30):
    kf = cv.KalmanFilter(18, 6, 0)
    state = np.zeros((18, 1), np.float32)
    
    # Transition matrix position/orientation
    tmp = np.eye(9, dtype=np.float32)
    tmp[0:3, 3:6] = np.eye(3, dtype=np.float32) * dt
    tmp[3:6, 6:9] = np.eye(3, dtype=np.float32) * dt
    tmp[0:3, 6:9] = np.eye(3, dtype=np.float32) * dt * dt / 2
    
    # Transition matrix
    kf.transitionMatrix = np.concatenate((
        np.concatenate((tmp, np.zeros((9, 9))), axis=1), 
        np.concatenate((np.zeros((9, 9)), tmp), axis=1)), 
        axis=0
    )
    
    # Measurement matrix
    tmp = np.zeros((6, 18), dtype=np.float32)
    tmp[0:3, 0:3] = np.eye(3, dtype=np.float32)
    tmp[3:6, 9:12] = np.eye(3, dtype=np.float32)
    kf.measurementMatrix = tmp
    
    # Process/Measurement noise covariance & Error
    kf.processNoiseCov = 1e-5 * np.eye(18, dtype=np.float32)
    kf.measurementNoiseCov = 1e-4 * np.eye(6, dtype=np.float32)
    kf.errorCovPost = np.eye(18, dtype=np.float32)
    
    return kf


filters = {
    'const_accell': const_accell
}

if __name__ == '__main__':
    print(const_accell(0.5))
