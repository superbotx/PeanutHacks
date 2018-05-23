from botX.components import BaseComponent

class ArmComponent(BaseComponent):
    def __init__(self):
        super(ArmComponent, self).__init__()

    def setup(self):
        """
        0. start the roslaunch
        1. make sure the moveit interface topic is in the list
        2. send one ping message
        3. wait until receive pong message
        4. return
        """

    def move_to(self, pose, server):
        pass

    def shutdown(self):
        """
        only need to terminate the roslaunch process
        """
