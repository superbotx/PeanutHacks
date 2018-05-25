#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Float64
import numpy as np
import moveit_commander

from peanut_moveit_interface.srv import MoveitInterface, GripperInterface

class Hackerman(object):

    def __init__(self):
        #### 0 - setup
        rospy.loginfo('waiting for services to come up...')
        rospy.wait_for_service('peanut_moveit_interface')
        rospy.wait_for_service('peanut_gripper_interface')
        self.execute_arm_move = rospy.ServiceProxy('peanut_moveit_interface', MoveitInterface)
        self.execute_gripper_move = rospy.ServiceProxy('peanut_gripper_interface', GripperInterface)
        rospy.loginfo('services reported!')

        # grasper = GraspPlanner()
        self.scene = moveit_commander.PlanningSceneInterface()
        p = PoseStamped()
        p.header.frame_id = "world"
        p.pose.position.x = 0
        p.pose.position.y = 0
        p.pose.position.z = 0
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 1
        self.scene.add_box("ground", p, (3, 3, 0.02))  # add a "ground plane"

    def just_do_it(self):
        #### 1 - get string input from user denoting an object
        object_to_pick = raw_input('please enter which object youd like to pick: ')
        rospy.loginfo('selected {} for picking'.format(object_to_pick))

        #### 2 - find bounding box for class
        # result is an array with 4 numbers, plus two images (color, depth)
        self.bbox = np.array([300, 200, 400, 300])

        #### 3 - find grasp within bounding box
        # pregrasp_pose, grasp_pose = grasper.get_grasp_plan(bbox, color_image, depth_image)

        #### 4 - execute grasp
        self.pose_msg = Pose()
        self.pose_msg.position.x = .371
        self.pose_msg.position.y = -.129
        self.pose_msg.position.z = .023
        self.pose_msg.orientation.x = 0.81
        self.pose_msg.orientation.y = 0.586
        self.pose_msg.orientation.z = -0.027
        self.pose_msg.orientation.w = 0.023

        result = raw_input('can I execute the grasp? (y/n): ')
        if result == 'y':
            # send to moveit
            try:
                self.execute_awesome_move_sequence(self.pose_msg)
            except ValueError:
                rospy.loginfo('we tried so hard, and got so far, but in the end, it didnt really matter :(')

    def execute_awesome_move_sequence(self, pose_msg):
        """pregrasp pose, grasp pose, open and close grippers etc"""
        rospy.loginfo('opening gripper...')
        self.open_gripper()

        rospy.loginfo('moving to pregrasp pose...')
        prepose_msg = self.create_prepose(pose_msg)
        self.move_arm_to(prepose_msg)

        result = raw_input('can I execute the grasp? (y/n): ')
        if result == 'y':
            rospy.loginfo('moving to grasp pose...')
            self.move_arm_to(pose_msg)
        else:
            raise ValueError('shoot')

        knees_weak = raw_input('can I close the gripper? (y/n): ')
        if knees_weak == 'y':
            rospy.loginfo('closing gripper')
            self.close_gripper()

        rospy.loginfo('now moving to end pose')
        endpose_msg = self.create_endpose()
        self.move_arm_to(endpose_msg)

        rospy.loginfo('done')


    def create_prepose(self, pose_msg):
        """create pose that sends robot to pose close to final grasp pose"""
        # for now hardcode
        prepose_msg = Pose()
        prepose_msg.orientation.x = 0.800
        prepose_msg.orientation.y = 0.599
        prepose_msg.orientation.z = -0.018
        prepose_msg.orientation.w = 0.031
        prepose_msg.position.x = 0.333
        prepose_msg.position.y = -0.114
        prepose_msg.position.z = 0.101

        return prepose_msg

    def create_endpose(self):
        prepose_msg = Pose()
        prepose_msg.orientation.x = 0.747
        prepose_msg.orientation.y = 0.647
        prepose_msg.orientation.z = 0.101
        prepose_msg.orientation.w = 0.113
        prepose_msg.position.x = 0.382
        prepose_msg.position.y = -0.057
        prepose_msg.position.z = 0.202

        return prepose_msg

    def move_arm_to(self, pose_msg):
        try:
            resp1 = self.execute_arm_move(pose_msg)
            rospy.loginfo('result: {}'.format(resp1))
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)

    def move_gripper_to(self, f1, f2, f3):
        unit_msg = String()
        unit_msg.data = 'percent'
        finger_position_msg = [Float64(f1), Float64(f2), Float64(f3)]
        try:
            resp2 = self.execute_gripper_move(unit_msg, finger_position_msg)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)

    def open_gripper(self):
        self.move_gripper_to(0,0,0)

    def close_gripper(self):
        self.move_gripper_to(100,100,100)


if __name__ == '__main__':
    rospy.init_node('hacking_now')
    rospy.loginfo('starting to hack')
    hm = Hackerman()
    hm.just_do_it()

