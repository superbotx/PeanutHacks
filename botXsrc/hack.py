from botX.tasks import BaseTask
from .mask_rcnn.ez import EZ

class HackTask(BaseTask):
    def __init__(self, robot):
        super(HackTask, self).__init__(robot)

    def setup(self):
        self.detector = EZ()

    def run(self):
        while True:
            try:
                object_name = input('Choose an object: ')
                curr_img = self.robot.components['camera'].get_img()
                object_dict = self.detector.detect(image_input=curr_img)
                if object_name in object_dict:
                    object_info = object_dict[object_name]
                    pose = self.robot.components['grasp_planner'].plan(object_info)
                    status = self.robot.components['arm'].move_to(pose)
                    if status == 'success':
                        print('done')
                    else:
                        print('cannot reach target')
                else:
                    print('Fuck, cannot find ', object_name)
            except:
                print('Whoops')
