#!/usr/bin/env python
import rospy

from moveit_interface import RobotPlanner, GripController
from grasp_planner import GQCNNPlanner
import numpy as np
import signal

from autolab_core import YamlConfig

import perception

from sensor_msgs.msg import Image

class PickPlaceDemo:
    def __init__(self, grasp_planner, path_planner, grip_controller, camera, config):
        self.grasp_planner = grasp_planner
        self.path_planner = path_planner
        self.grip_controller = grip_controller

        self.config = config
        self.camera = camera

    def _plan_and_execute(self, location, pause_message=None):
        """ plan and execute a path to a location with optional paus message
        """
        self.path_planner.plan(location)
        if pause_message:
            data = raw_input(pause_message)
            if data == 'q':
                return False
        return self.path_planner.execute()

    def _get_bounding_box(self, object_name, color_image):
        """ Find the bounding box for the object
        params
        ---
        object_name: the string name of the object
        color_image: the color image for the object

        returns
        ---
        bounding_box: numpy array [minX, minY, maxX, maxY] in pixels around the image 
        on the depth image
        """
        return np.array([300,200,400,300]) 

if __name__ == '__main__':
    rospy.init_node("pick_place_demo", log_level=rospy.DEBUG)
    config = YamlConfig('/home/baymax/catkin_ws/src/jaco_manipulation/cfg/grasp_test.yaml')

    # create rgbd sensor
    rospy.loginfo('Creating RGBD Sensor')
    sensor_cfg = config['sensor_cfg']
    sensor_type = sensor_cfg['type']
    camera = perception.RgbdSensorFactory.sensor(sensor_type, sensor_cfg)
    camera.start()
    rospy.loginfo('Sensor Running')
    
    # setup safe termination
    def handler(signum, frame):
        rospy.loginfo('caught CTRL+C, exiting...')        
        # if camera is not None:
        #     # camera.stop()
        #TODO fix subscriber to be real      
        exit(0)
    signal.signal(signal.SIGINT, handler)

    frame = config['sensor_cfg']['frame']

    camera_intrinsics = camera.ir_intrinsics
    #camera_intrinsics = perception.CameraIntrinsics(frame, fx=365.46, fy=365.46, cx=254.9, cy=205.4, skew=0.0, height=424, width=512) #TODO set height and width with param 
    grasp_planner = GQCNNPlanner(camera_intrinsics, config)

    path_planner = RobotPlanner()
    grip_controller = GripController()
    object_name = "cup"

    picker = PickPlaceDemo(grasp_planner, path_planner, grip_controller, camera, config)
    picker.move_object(object_name)

    rospy.spin()