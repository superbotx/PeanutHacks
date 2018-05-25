## Arm calibration code for peanut robotics
In order to calibrate the robot place an AR tag on the robot as follows.

The code will look for AR tag 6 by default.

Once the code has located the AR tag it will lookup the transformation between the AR tag and the camera frame and then publlish a transformation between the camera and the robot continuously using tf.

The following command will launch all of this:

`roslaunch peanut_calibration arm_calibration.launch `