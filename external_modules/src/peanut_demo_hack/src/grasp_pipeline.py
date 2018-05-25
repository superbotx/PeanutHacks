#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np

from peanut_moveit_interface.srv import *

def main():
    #### 0 - setup
    rospy.wait_for_service('')

    #### 1 - get string input from user denoting an object
    object_to_pick = raw_input()


    #### 2 - find bounding box for class
    # result is an array with 4 numbers, plus two images

    #### 3 - find grasp within bounding box

    #### 4 - execute grasp
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

# wpose = geometry_msgs.msg.Pose()
# orientation = np.array([1, 0, 1, 0])
# orientation = orientation / np.linalg.norm(orientation)  # enforce unit quaternion
# wpose.orientation.x = orientation[0]
# wpose.orientation.y = orientation[1]
# wpose.orientation.z = orientation[2]
# wpose.orientation.w = orientation[3]
# wpose.position.x = 0.4
# wpose.position.y = 0.0
# wpose.position.z = 0.1

if __name__ == '__main__':
    rospy.init_node('hacking_now')
    rospy.loginfo('starting to hack')
    main()