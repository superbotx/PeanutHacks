from botX.tasks import BaseTask

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
