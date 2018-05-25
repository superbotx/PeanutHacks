#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Float64
import numpy as np

from peanut_moveit_interface.srv import MoveitInterface, GripperInterface

def main():
    #### 0 - setup
    rospy.loginfo('waiting for services to come up...')
    rospy.wait_for_service('peanut_moveit_interface')
    rospy.wait_for_service('peanut_gripper_interface')
    execute_arm_move = rospy.ServiceProxy('peanut_moveit_interface', MoveitInterface)
    execute_gripper_move = rospy.ServiceProxy('peanut_gripper_interface', GripperInterface)
    rospy.loginfo('services reported!')

    #### 1 - get string input from user denoting an object
    object_to_pick = raw_input('please enter which object youd like to pick: ')
    rospy.loginfo('selected {} for picking'.format(object_to_pick))

    #### 2 - find bounding box for class
    # result is an array with 4 numbers, plus two images

    #### 3 - find grasp within bounding box

    #### 4 - execute grasp
    pose_msg = Pose()

    pose_msg.position.x = .2
    pose_msg.position.y = -.270
    pose_msg.position.z = .48
    pose_msg.orientation.x = 0.67
    pose_msg.orientation.y = -0.15
    pose_msg.orientation.z = -0.69
    pose_msg.orientation.w = 0.17

    unit_msg = String()
    unit_msg.data = 'percent'
    finger_position_msg = [Float64(50), Float64(50), Float64(50)]

    try:
        resp1 = execute_arm_move(pose_msg)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
    try:
        resp2 = execute_gripper_move(unit_msg, finger_position_msg)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == '__main__':
    rospy.init_node('hacking_now')
    rospy.loginfo('starting to hack')
    main()