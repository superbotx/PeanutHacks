#! /usr/bin/env python
"""Demoscrip for testing the moveit_interface with moveit without botx"""

import rospy

from geometry_msgs.msg import Pose

from peanut_moveit_interface.srv import *


if __name__ == '__main__':
    # create a dummy pose to send to moveit
    pose_msg = Pose()
    pose_msg.position.x = 1
    pose_msg.position.y = 1
    pose_msg.position.z = 1
    pose_msg.orientation.x = 0
    pose_msg.orientation.y = 0
    pose_msg.orientation.z = 0
    pose_msg.orientation.w = 1

    rospy.wait_for_service('peanut_moveit_interface')

    try:
        actually_move_it = rospy.ServiceProxy('peanut_moveit_interface', MoveitInterface)
        response = actually_move_it(pose_msg)
        rospy.loginfo("response was: {}".format(response))
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


