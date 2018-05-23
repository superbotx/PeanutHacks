from botX.tasks import BaseTask

class PeanutHacksDemo(BaseTask):
    def __init__(self, robot):
        super(PeanutHacksDemo, self).__init__(robot)
        # self.add_subtask('locate_object', LocateObject(robot))

    def setup(self, **kwargs):
        # anything this task needs to know upfront will be set here
        pass

    def run(self, **kwargs):
        """
        this method will perform the pick and place, by calling
        self.get_subtask('...')
        """
        # self.get_subtask('locate_object').run(target_object='cup')

        pass