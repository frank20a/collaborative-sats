import numpy as np 
import cv2

# Creating an object of StereoBM algorithm
stereo = cv2.StereoBM_create()

# Capturing and storing left and right camera images
imgL= cv2.imread('im2.png')
imgR= cv2.imread('im6.png')

# Camera Characteristics
B = 9           # Distance between the cameras (cm)
f = 6           # Camera lense focal length (mm)
fov = 56.6      # Camera field of view (degrees)

height_r, width_r, depth_r = imgR.shape
height_l, width_l, depth_l = imgL.shape
f_pixel = (width_r / 2) / np.tan(fov * .5 * np.pi / 180)

while True:
    Right_nice = cv2.cvtColor(imgR,cv2.COLOR_BGR2GRAY)
    Left_nice = cv2.cvtColor(imgL,cv2.COLOR_BGR2GRAY)
    

    # Setting the updated parameters before computing disparity map
    minDisparity = 0
    numDisparities = 16 * 4

    stereo.setMinDisparity(minDisparity)
    stereo.setNumDisparities(numDisparities)
    stereo.setBlockSize(7)
    stereo.setUniquenessRatio(10)
    stereo.setSpeckleRange(4)
    stereo.setSpeckleWindowSize(10)
    stereo.setDisp12MaxDiff(1)

    # Calculating disparity using the StereoBM algorithm
    disparity = stereo.compute(Left_nice, Right_nice)
    # NOTE: Code returns a 16bit signed single channel image,
    # CV_16S containing a disparity map scaled by 16. Hence it 
    # is essential to convert it to CV_32F and scale it down 16 times.

    # Converting to float32 
    disparity = disparity.astype(np.float32)

    # Scaling down the disparity values and normalizing them 
    disparity = (disparity/16.0 - minDisparity)/numDisparities

    # Displaying the disparity map
    cv2.imshow("disp",disparity)

    # Close window using esc key
    if cv2.waitKey(1) == 27:
        break
