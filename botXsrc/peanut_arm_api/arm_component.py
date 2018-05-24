from botX.components import BaseComponent
from botX.applications import external_command_pool, botXimport

class ArmComponent(BaseComponent):
    def __init__(self):
        super(ArmComponent, self).__init__()

    def setup(self):
        """
        1. TODO make sure the moveit interface topic is in the list
        """
        # Initialize rosbridge
        self.server = botXimport('rosbridge')['rosbridge_suit_component']['module']()
        self.server.setup()

    def move_to(self, pose):
        """
        Move arm to given pose
        :param pose:
        :return:
        """
        service = '/peanut_moveit_interface'
        self.server.call_service(service, callback=self.move_response_cb, args=[pose])

    def move_response_cb(self, msg):
        """Handle response received from peanut_moveit_interface"""
        # store in buffer?
        if msg.data == 'failed':
            raise ValueError('execution failed')
        return

    def shutdown(self):
        """
        only need to terminate the roslaunch process
        """
        pass

