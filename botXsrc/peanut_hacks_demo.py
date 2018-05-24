from botX.tasks import BaseTask
from peanut_camera_api.peanut_camera_api import CameraAPI
import mask_rcnn.ez as ez
from PIL import Image
import io

import base64
import numpy as np


class PeanutHacksDemo(BaseTask):
    def __init__(self, robot):
        super(PeanutHacksDemo, self).__init__(robot)
        # self.add_subtask('locate_object', LocateObject(robot))

    def setup(self, **kwargs):
        # anything this task needs to know upfront will be set here

        self.camera = CameraAPI()
        # self.camera.setup()
        self.mask_rcnn = ez.EZ()

        pass

    def run(self, **kwargs):
        self.camera.setup_subscribers()
        im = self.camera.get_color_image()
        print("----------- IMAGE HERE ------------")
        print(im.keys())
        print(im['encoding'])
        print("height: ", im['height'])
        print("width: ", im['width'])
        print("header", im['header'])

        im_d = self.camera.get_depth_image()
        print("----------- DEPTH IMAGE HERE ------------")
        print(im_d['encoding'])
        print("height: ", im_d['height'])
        print("width: ", im_d['width'])
        print("header", im_d['header'])

        print("size: ", im['width']*im['height'])
        print("length: ", len(im['data']))

        # text_file = open("Output.txt", "w")
        # text_file.write(im['data'])
        # text_file.close()

        # text_file = open("FullOutput.txt", "w")
        # text_file.write(str(im))
        # text_file.close()

        byte_image = im['data'].encode('utf-8')
        decoded_image = base64.decodebytes(byte_image)
        image = Image.frombytes('RGB', (640, 480), decoded_image)
        image.show()
        im_result, info = self.mask_rcnn.detect(image)
        print(info)
        im_result.show()




        # im_bytes = im['data'].encode('utf-8')
        # print(im_bytes)

        # img = Image.open(io.BytesIO(im_bytes))
        # print (type(img))

        # t = np.arange(25, dtype=np.float64)
        # s = base64.b64encode(t)
        # r = base64.decodestring(s)
        # q = np.frombuffer(r, dtype=np.float64)

        # TODO 
        #  * run through mask_rcnn and return bounding box

        # annotated_img, statistic_res = self.mask_rcnn.detect(images[0])
        # if target_object in set(statistic_res['class_names']):
        #     return 'found', statistic_res[target_object]
        # else:
        #     return 'not_found', None


        """
        this method will perform the pick and place, by calling
        self.get_subtask('...')
        """
        # self.get_subtask('locate_object').run(target_object='cup')

        pass