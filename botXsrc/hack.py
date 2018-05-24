from botX.tasks import BaseTask
from .mask_rcnn.ez import EZ
from .peanut_arm_api.arm_component import ArmComponent
from .peanut_camera_api.peanut_camera_api import CameraAPI

config = {
    'environment': 'robot'
}

class HackTask(BaseTask):
    def __init__(self, robot):
        super(HackTask, self).__init__(robot)

    def setup(self):
        pass

    def run(self):
        while True:
            try:
                object_name = input('Choose an object: ')

            except:
                print('Whoops')
