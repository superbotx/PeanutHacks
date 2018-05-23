from botX.robots import BaseRobot
from botX.components import BaseComponent
from botX.applications import botXimport

class PeanutHacksBot(BaseRobot):

    def __init__(self):
        super(PeanutHacksBot, self).__init__()

        # self.add_component('camera',GazeboSimKinect())


    def additional_setup(self):
        # do any additional setup other than the ones in component setup
        pass

    def additional_shutdown(self):
        # do any additional shutdown actions
        pass