from botX.components import BaseComponent
from botX.applications import external_command_pool, botXimport

class GripperComponent(BaseComponent):
    def __init__(self):
        super(GripperComponent, self).__init__()

    def setup(self):
        """
        1. TODO make sure the moveit interface topic is in the list
        """
        # Initialize rosbridge
        self.server = botXimport('rosbridge')['rosbridge_suit_component']['module']()
        self.server.setup()
        # self.server.advertise_service('peanut_moveit_interface/GripperInterface', '/peanut_gripper_interface')

    def gripper_to(self, units, finger_positions):
        """
        Move arm to given pose
        :param units: string: mm, percent, ...
        :param finger_positions: [float, float, float]
        """
        service = '/peanut_gripper_interface'
        self.server.call_service(service, callback=self.gripper_response_cb, args=[units, finger_positions])

    def gripper_response_cb(self, msg):
        """Handle response received from peanut_moveit_interface"""
        if msg.data == 'failed':
            raise ValueError('execution failed')
        return

    def shutdown(self):
        """
        only need to terminate the roslaunch process
        """
        pass

