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
        self.server = botXimport('rosbridge_api')['rosbridge_suit_component']['module']()
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

ac = ArmComponent()
my_pose = dict()
my_pose['position'] = {'x':0.2, 'y':-0.27, 'z':0.48}
my_pose['orientation'] = {'x':0.67, 'y':-0.15, 'z':-0.69, 'w':0.17}
try:
    ac.move_to(my_pose)
except ValueError as e:
    print e