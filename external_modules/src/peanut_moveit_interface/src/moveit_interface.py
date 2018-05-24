import arm_srv
import rospy
from util import *
import numpy as np

import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose

class MoveitInterface(object):
    """Interface between higher lvl code operating through botx and moveit
    requires moveit to be launched and controllers to be up
    """

    def __init__(self):

        # this variable is set when a service is being handled
        self.processing_srv = False

        # start the moveit commander
        moveit_commander.roscpp_initialize(sys.argv)

        # create robot commander instance
        self.robot = moveit_commander.RobotCommander()

        # initialize the planning scene
        self.scene = moveit_commander.PlanningSceneInterface()

        # initialize the arm move group
        self.group = moveit_commander.MoveGroupCommander("arm")

        # for publishing trajectories to Rviz for visualization
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

    def execute_pose(self, pose_msg):
        """
        send the pose to moveit
        :param pose_msg: geometry_msgs/Pose
        :return string succesa or failed (for now):
        """
        self.group.clear_pose_target()
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(pose_msg)
        found_plan = self.group.plan()

        if found_plan:
            # block for now
            # self.group.execute(self.found_plan, wait=False)
            self.group.execute(found_plan, wait=False)
            status = 'success'
        else:
            rospy.logerror("Couldn't find a plan to goal so not executing")
            status = 'failed'

        self.group.clear_pose_target()

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
            # xyz = np.array([req.position.x, req.position.y, req.position.z])
            # quat = np.array([req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w])
            result = self.execute_pose(req)

        return MoveitInterfaceResponse(result)






# def handler(req):
#     if curr_srv:
#         send_terminate_response(curr_srv)
#     setup_group(group, req)
#     group.go(wait=False)
#     group.onFinish(callback=send_finish_response)

if __name__ == "__main__":
    rospy.init_node('moveit_interface')
    moveit_interface = MoveitInterface()
    s = rospy.Service('moveit_interface', arm_srv, moveit_interface.higherup_handler)
    rospy.spin()