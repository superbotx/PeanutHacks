#!/usr/bin/env python
import sys
import rospy

import geometry_msgs.msg
import shape_msgs.msg
import std_msgs.msg

import kinova_msgs.msg
import actionlib
import numpy as np

class GripperController:
    """
    The kinova hand uses an action server for grip actions.
    """

    def __init__(self):
        self.finger_maxDist = 18.9 / 2 / 1000  # max distance for one finger
        self.finger_maxTurn = 6800  # max thread rotation for one finger

        action_address = '/m1n6s300_driver/fingers_action/finger_positions'

        self.grip_action_client = actionlib.SimpleActionClient(action_address,
                                                               kinova_msgs.msg.SetFingersPositionAction)

    def grip(self, units, finger_positions):
        """
        Params
        ---
        units: a string representing units (turn, mm, percent)
        finger_values: an array like object of dim 3
        Returns
        ---
        None if the grip timed out
        """
        turn, meter, percent = self._convert_units(units, finger_positions)
        positions_temp1 = [max(0.0, n) for n in turn]
        positions_temp2 = [min(n, self.finger_maxTurn) for n in positions_temp1]
        positions = [float(n) for n in positions_temp2]
        return self._execute_grip(turn)

    def _execute_grip(self, finger_positions):
        """ move the gripper to the finger position
        """
        if len(finger_positions) != 3:
            raise ValueError("wrong number of finger specified")

        print("wating for action client")
        self.grip_action_client.wait_for_server()

        goal = kinova_msgs.msg.SetFingersPositionGoal()
        goal.fingers.finger1 = float(finger_positions[0])
        goal.fingers.finger2 = float(finger_positions[1])
        goal.fingers.finger3 = float(finger_positions[2])
        print("sending grip goal {}".format(goal))
        self.grip_action_client.send_goal(goal)
        if self.grip_action_client.wait_for_result(rospy.Duration(5.0)):
            return self.grip_action_client.get_result()
        else:
            self.grip_action_client.cancel_all_goals()
            rospy.logwarn('        the gripper action timed-out')
            raise ValueError('gripper action timed out')

    def _convert_units(self, unit_, finger_value_):
        """
        converts units from the input unit (unit_) to all units (turn_, meter_, percent_)
        ---
        unit_: a string representing the input units (turn, mm, percent)
        finger_value_: a  value associated with the grasp
        Returns
        ---
        a tuple containing all units turn units, finger meter, finger percent
        """
        # transform between units
        if unit_ == 'turn':
            # get absolute value
            finger_turn_ = finger_value_
            finger_meter_ = [x * self.finger_maxDist / self.finger_maxTurn for x in finger_turn_]
            finger_percent_ = [x / self.finger_maxTurn * 100.0 for x in finger_turn_]

        elif unit_ == 'mm':
            # get absolute value
            finger_turn_ = [x / 1000 * self.finger_maxTurn / self.finger_maxDist for x in finger_value_]
            finger_meter_ = [x * self.finger_maxDist / self.finger_maxTurn for x in finger_turn_]
            finger_percent_ = [x / self.finger_maxTurn * 100.0 for x in finger_turn_]

        elif unit_ == 'percent':
            # get absolute value
            finger_turn_ = [x / 100.0 * self.finger_maxTurn for x in finger_value_]
            finger_meter_ = [x * self.finger_maxDist / self.finger_maxTurn for x in finger_turn_]
            finger_percent_ = [x / self.finger_maxTurn * 100.0 for x in finger_turn_]
        else:
            raise Exception("Finger value have to be in turn, mm or percent")

        return finger_turn_, finger_meter_, finger_percent_

if __name__ == '__main__':
    rospy.init_node('standalone_test_gripper')
    g = GripperController()
    g.grip(units='percent', finger_positions=[50,50,50])