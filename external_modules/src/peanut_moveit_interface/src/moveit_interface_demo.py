#! /usr/bin/env python
"""Demoscrip for testing the moveit_interface with moveit without botx"""

import rospy

from geometry_msgs.msg import Pose

from peanut_moveit_interface.srv import *


if __name__ == '__main__':
    rospy.init_node('testing_moveit_interface')

    # create a dummy pose to send to moveit
    pose_msg = Pose()

    pose_msg.position.x = .2
    pose_msg.position.y = -.270
    pose_msg.position.z = .48
    pose_msg.orientation.x = 0.67
    pose_msg.orientation.y = -0.15
    pose_msg.orientation.z = -0.69
    pose_msg.orientation.w = 0.17

    rospy.loginfo('connecting to interface service...')
    rospy.wait_for_service('peanut_moveit_interface')
    rospy.loginfo('found interface service!')
    try:
        actually_move_it = rospy.ServiceProxy('peanut_moveit_interface', MoveitInterface)
        response = actually_move_it(pose_msg)
        rospy.loginfo("response was: {}".format(response.result.data))
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


