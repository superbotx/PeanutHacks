#!/usr/bin/env python

import numpy as np
import tf
import rospy
import signal
import copy

import spacial_location
from spacial_location import Pose #TODO replace this with full reference, too many poses to use simply Pose

from std_msgs.msg import Header
from gqcnn.srv import GQCNNGraspPlanner
from gqcnn.msg import GQCNNGrasp, BoundingBox
import geometry_msgs.msg
import std_msgs.msg
from sensor_msgs.msg import Image, CameraInfo

from peanut_moveit_interface.srv import *

import perception
from perception import CameraIntrinsics, ColorImage, DepthImage
from perception import RgbdDetectorFactory, RgbdSensorFactory

from autolab_core import YamlConfig, RigidTransform
from autolab_core import Box
# import autolab_core.rigid_transformations.RigidTransform as RigidTransform

from cv_bridge import CvBridge, CvBridgeError

from gqcnn import Visualizer as vis

import cv2
import matplotlib.pyplot as plt
import matplotlib.pyplot as pyplot


class GraspPlanner:
    """ Abstract clss for grasp planning.
    """
    def __init__(self):
        self.listener = tf.TransformListener()
    
    def get_grasp_plan(self, object_name):
        assert False

class GQCNNPlanner(GraspPlanner):
    """ pass an image to the GQCNN
    """
    def __init__(self):
        # wait for Grasp Planning Service and create Service Proxy
        rospy.loginfo("Waiting for GQCNN to spin up")
        rospy.wait_for_service("plan_gqcnn_grasp") #TODO put this into botX
        self.plan_grasp = rospy.ServiceProxy('plan_gqcnn_grasp', GQCNNGraspPlanner)
        rospy.loginfo("GQCNN service Initialized")

        self.listener = tf.TransformListener()

        self.config = YamlConfig('/home/baymax/catkin_ws/src/jaco_manipulation/cfg/grasp_test.yaml')

        self.frame = self.config['sensor_cfg']['frame']

    def _bbox_to_msg(self, bbox):
        """
        Params
        ---
        bbox: numpy array [minX, minY, maxX, maxY] in pixels around the image 
        Returns
        ---
        a bondingBox message type
        """       
        boundingBox = BoundingBox()
        boundingBox.minX = bbox[0]
        boundingBox.minY = bbox[1]
        boundingBox.maxX = bbox[2]
        boundingBox.maxY = bbox[3]
        return boundingBox

    def _get_new_pose(self, pose_stamped, target_frame="/world"):
        """ Transform a pose from its frame to the desired frame
        Params
        ----
        pose_stamped: A ros stamped pose with its reference frame specified
        target_frame: The desired frame of the robot
        Returns
        ----
        pose_world_stamped: The pose in the world frame
        """
        #TODO this should be changed to a world frame
        self.listener.waitForTransform(target_frame, self.camera_intrinsics.frame, rospy.Time(0), rospy.Duration(4.0))
        try:
            pose_world_stamped = self.listener.transformPose(target_frame, pose_stamped)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("tf was not able to find a transformation between the world frame and the grasp point")
            return False

        return pose_world_stamped

    def _get_transformation(self, from_frame, to_frame="world"):
        """ Find the transformation from one frame to another in ROS 
        """
        #TODO this should be contained in a transformation class
        target_frame = to_frame
        source_frame = from_frame
        self.listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(4.0))
        try:
            pos, quat_out = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("tf was not able to find a transformation from {} to {}".format(from_frame, to_frame))
            return False
        #import pdb; pdb.set_trace()
        quat = [quat_out[3], quat_out[0], quat_out[1], quat_out[2]] #the quaternion of ros has a different representation than other sys
        return RigidTransform(quat, pos, from_frame, to_frame)
        
    def get_grasp_plan(self, bounding_box, color_image, depth_image):
        """ finds the highest quality score grasp associated with the object

        """
        #grab frames for depth image and color image
        rospy.loginfo("grabbing frames")

        boundingBox = self._bbox_to_msg(bounding_box)

        vis = False
        if vis:
            pyplot.figure()
            pyplot.subplot(2,2,1)
            pyplot.title("color image")
            pyplot.imshow(color_image.data)

            pyplot.subplot(2,2,2)
            pyplot.title("depth image")
            pyplot.imshow(depth_image.data)

        # inpaint to remove holes
        inpainted_color_image = color_image.inpaint(self.config["inpaint_rescale_factor"]) #TODO make rescale factor in config
        inpainted_depth_image = depth_image.inpaint(self.config["inpaint_rescale_factor"])
        
        if vis:
            pyplot.subplot(2,2,3)
            pyplot.title("inpaint color")
            pyplot.imshow(inpainted_color_image.data)

            pyplot.subplot(2,2,4)
            pyplot.title("inpaint depth")
            pyplot.imshow(inpainted_depth_image.data)
            pyplot.show()
        
        try:
            rospy.loginfo("Sending grasp plan request to gqcnn server")
            planned_grasp_data = self.plan_grasp(inpainted_color_image.rosmsg, inpainted_depth_image.rosmsg, self.camera_intrinsics.rosmsg, boundingBox)

            planned_grasp_pose_msg = planned_grasp_data.grasp.pose
            grasp_succes_prob = planned_grasp_data.grasp.grasp_success_prob
            grasp = planned_grasp_data.grasp

            rospy.loginfo("Grasp service request response: {}".format(planned_grasp_data))

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: \n %s" % e)  

        rotation_quaternion = np.asarray([grasp.pose.orientation.w, grasp.pose.orientation.x, grasp.pose.orientation.y, grasp.pose.orientation.z]) 
        translation = np.asarray([grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z])
        T_grasp_camera = RigidTransform(rotation_quaternion, translation, "grasp", self.camera_intrinsics.frame)

        T_world_camera = self._get_transformation(self.camera_intrinsics.frame) #TODO this should be a rigidbody.getfromtransform call
        
        rotation = np.array([[0, 0, 1],[0, 1, 0],[-1, 0, 0]])
        translation = np.array([0, 0, 0])
        T_grasp_gripper = RigidTransform(rotation, translation, from_frame="gripper", to_frame="grasp")
        T_grasp_world = T_world_camera * T_grasp_camera * T_grasp_gripper

        import pdb; pdb.set_trace()
        t_quat = [T_grasp_world.quaternion[1], T_grasp_world.quaternion[2], T_grasp_world.quaternion[3], T_grasp_world.quaternion[0]]
        grasp_pose_world = Pose(T_grasp_world.position, t_quat, frame="/world")

        pre_grasp_pose = offset_hand(grasp_pose_world, unit_vector=[0,0,1])

        return pre_grasp_pose, grasp_pose_world

def mask_to_bounding_box(masked_image):
    """ Convert a masked image to a bounding box
        params
        ---
        masked_image: #TODO add in description

        returns
        ---
        bounding_box: numpy array [minX, minY, maxX, maxY] in pixels around the image 
        on the depth image
    """
    #TODO actually convert between the masked image and the bounding box
    return [300,200,400,300]

def offset_hand(pose_input, unit_vector=[1, 0, 0] ,offset_dist=0.1):
    """ Find the pre grasp pose by offsetting the hand backwards from its current position
    grasp_pose: the pose of the grasp location
    offset_dist: the amount to offset off the object
    Returns
    ---
    pre_grasp_pose: the offsetted pose of the object
    """
    #use the unit z vector because thats the direction out of the hand
    quat = pose_input.orientation[0:4]

    quat_inv = tf.transformations.quaternion_inverse(quat)
    direction = spacial_location.qv_mult(quat, unit_vector)
    pre_grasp_position = pose_input.position - direction*offset_dist

    pre_grasp_pose = Pose(pre_grasp_position, pose_input.orientation)

    pose_input.show_position_marker(ident = 1, label = "grasp pose")
    pre_grasp_pose.show_position_marker(ident = 2, label = "pregrasp pose")
    # import pdb; pdb.set_trace()

    return pre_grasp_pose


if __name__ == "__main__":
    rospy.init_node('moveit_interface')
    moveit_interface = MoveitInterfacer()

    bounding_box = mask_to_bounding_box(req.maskImage)

    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    perception.ColorImage(data, frame = self.frame)
    perception.DepthImage(data, frame = self.frame)

    #TODO color image and depth image need to be perception objects
    result = self.get_grasp_plan(bounding_box, req.colorImage, req.depthImage)

    rospy.spin()