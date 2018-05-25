#! /usr/bin/env python
import rospy
import numpy as np

import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from gripper_client import GripperController
from peanut_moveit_interface.srv import *


class MoveitInterfacer(object):
    """Interface between higher lvl code operating through botx and moveit
    requires moveit to be launched and controllers to be up
    """

    def __init__(self):

        # this variable is set when a service is being handled
        self.processing_srv = False

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_controller = GripperController()

        # for publishing trajectories to Rviz for visualization
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # TODO replace with an action service once botx can handle that
        rospy.Service('peanut_moveit_interface', MoveitInterface, self.higherup_handler)
        rospy.Service('peanut_gripper_interface', GripperInterface, self.gripper_higherup_handler)

    def execute_pose(self, pose_msg):
        """
        send the pose to moveit
        :param pose_msg: geometry_msgs/Pose
        :return string success or failed (for now):
        """
        self.group.clear_pose_targets()
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(pose_msg)
        found_plan = self.group.plan()

        if found_plan.joint_trajectory.joint_names != []:
            # block for now
            # self.group.execute(self.found_plan, wait=False)
            self.group.execute(found_plan, wait=True)
            status = 'success'
        else:
            rospy.logerr("Couldn't find a plan to goal so not executing")
            status = 'failed'

        self.group.clear_pose_targets()

        rospy.loginfo(status)
        return status

    def higherup_handler(self, req):
        """handle an incoming request
        :param req: Pose msg
        """
        if self.processing_srv:
            # TODO decide what to do (based on the request contents?)
            # for now terminate
            raise NotImplementedError('woops lol')
            # self.group.TERMINATE  # TODO

        else:  # nothing going on, execute the request
            result = self.execute_pose(req.goal_pose)

        response = MoveitInterfaceResponse()
        response.data = result
        return response

    def gripper_higherup_handler(self, req):
        """handle an incoming request
         :param req: GripperInterface.srv
         """
        if self.processing_srv:
            # TODO decide what to do (based on the request contents?)
            # for now terminate
            raise NotImplementedError('woops lol')
            # self.group.TERMINATE  # TODO

        else:  # nothing going on, execute the request
            posns = [int(pos.data) for pos in req.finger_positions]
            try:
                self.gripper_controller.grip(req.units.data, posns)
                result = 'success'
            except ValueError:
                result = 'failed'
            rospy.loginfo(result)

        response = String()
        response.data = 'failed'
        return response

if __name__ == "__main__":
    rospy.init_node('moveit_interface')
    moveit_interface = MoveitInterfacer()
    rospy.spin()
