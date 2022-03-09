# Pose Estimation 

## Nodes

### calibrate
Generates a calibration.json file that can be used to un-distort a camera output. The calibration includes holding a chessboard image in front of the camera and pressing the space bar to take a calibration picture. Set the appropriate parameter to at least 10 images of the chessboard. 

#### Parameters
- `num-images` Number of calibration images
- `chessboard-h` Height of the chessboard (number of inside vertices = squares-1)
- `chessboard-w` Width of the chessboard (number of inside vertices = squares-1)

### undistort
Takes the calibration.json file and un-distorts the computers camera input. It then publishes the un-distorted video frame-by-frame to the /undistorted topic.

#### Parameters
- `verbose` Verbocity level (0-Nothing, 1-Image Preview)

### aruco_pose_estimation
Takes the output of the undistort node in the /undistorted topic and calculates the pose of any markers relative to the camera frame

#### Parameters
- `verbose` Verbocity level (0-Nothing, 1-Publish image with pose estimation on /aruco_verbose, 2-Image Preview, 3-Translation & Rotation matrix in the console)