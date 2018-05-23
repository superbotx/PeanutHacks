from botX.tasks import BaseTask
from peanut_camera_api.peanut_camera_api import CameraAPI

class PeanutHacksDemo(BaseTask):
    def __init__(self, robot):
        super(PeanutHacksDemo, self).__init__(robot)
        # self.add_subtask('locate_object', LocateObject(robot))

    def setup(self, **kwargs):
        # anything this task needs to know upfront will be set here

        self.camera = CameraAPI()
        # self.camera.setup()

        pass

    def run(self, **kwargs):
        # self.camera

        """
        this method will perform the pick and place, by calling
        self.get_subtask('...')
        """
        # self.get_subtask('locate_object').run(target_object='cup')

        pass