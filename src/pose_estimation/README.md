# Pose Estimation 

The `pose_estimation` package provides various methods for a chaser to estimate the pose of a target.

Flow of the pose estimation:

- Chaser publishes image from camera
- Undistort ~45ms
- Estimate pose from markers ~25ms
- Translate and publish odometry from chasers 
- Combine estimations from different chasers ~5ms

Total average time: ~75ms


## calibrator
Generates a calibration.json file that can be used to un-distort a camera output. The calibration includes holding a chessboard image in front of the camera and ~pressing the space bar~ to take a calibration picture. Set the appropriate parameter to at least 10 images of the chessboard. 

### Parameters
- int `num-images` Number of calibration images (default: 15)
- int `chessboard-h` Height of the chessboard (number of inside vertices = squares-1, default: 8)
- int `chessboard-w` Width of the chessboard (number of inside vertices = squares-1, default: 5)
- str `type` Can be "live" or "file". It will use either input from the computer camera or images from a folder (default: 'live')
- str `folder` The folder containing the images if `type` is "file" (default: '/home/frank20a/calibration_imgs')

## sim_calibrator
Generates a sim_calibration.json file that can be used to un-distort a camera output. The calibration process is automatic as the script will spawn a chessboard in the Gazebo simulation environment and with move it arrount in front of the robots camera taking pictures. This will take some time but in the end the script will stop analising images and stay in an infinite loop moving arround the chessboard fast. Then you can close it. Keep in mind the issue #2.

### Parameters
- int `num-images` Number of calibration images (default: 15)
- int `chessboard-h` Height of the chessboard (number of inside vertices = squares-1, default: 7)
- int `chessboard-w` Width of the chessboard (number of inside vertices = squares-1, default: 7)

## undistort
Takes a cameras calibration parameters from a JSON file and un-distorts images taken by that camera. It then publishes the un-distorted video frame-by-frame to the /undistorted topic. The node either uses the computers camera input or a ROS topic on which a simulation publishes images.

Duration per image: ~0.00ns

### Parameters
- int `verbose` Verbocity level (0-Nothing, 1-Image Preview, default: 0)
- bool `sim` Whether the node is undistorting data from a simulation or the computer camera (default: False)
- str `camera-name` The name of the camera in the gazebo simulation (default: '/front_camera')

## aruco_estimator
Takes the output of the undistort node in the /undistorted topic and calculates the pose of any markers relative to the camera frame. It uses the 5x5_50 ArUco Dictionary.

### Parameters
- `verbose` Verbocity level (0-Nothing, 1-Publish image with pose estimation on /aruco_verbose, 2-Image Preview, 3-Translation & Rotation matrix in the console, default: 1)
- `marker_size` the real size of the printed marker in meters (default: 0.12)
- `sim` Whether the node is processing data from a simulation or the computer camera (default: False)

## aruco_board_estimator
This node uses the ArUco Board functionality of OpenCV to calculate the pose of a 3D object that has ArUco markers (from the 5x5_50 Dictionary) printed on it. For that, prio knowledge of the position of the markers relative to the objects axis is required. This knowledge must be given in the form of OpenCV.ArUco Object Points which is a 4x3 array containing the position of each corner (in **CCW** order) of each marker on the object relative to the objects axis. These models are written in the [`objPoints.py`](/pose_estimation/objPoints.py) following the example of `marker_cube_1`.

Duration per image: ~20-30ms

### Parameters
- `verbose` Verbocity level (0-Nothing, 1-Publish image with pose estimation on /aruco_verbose, 2-Image Preview, 3-Translation & Rotation matrix in the console, default: 1)
- `sim` Whether the node is processing data from a simulation or the computer camera (default: False)
- `model` then name of the model, written in [`objPoints.py`](/pose_estimation/objPoints.py) and inserted into the `models` dictionary.

## Combine Estimations
This node combines the pose estimation of the target from the multiple chasers by averaging them. It publishes the pose of the object relative to the camera frame.

Duration per cycle: ~1-12ms

### Parameters
- `fps` The number of cycles per second to calculate the average pose (default: 50)