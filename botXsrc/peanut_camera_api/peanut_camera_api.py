from botX.components import BaseComponent
from botX.applications import external_command_pool, botXimport

import time

class CameraAPI(BaseComponent):

    def setup(self):
        """
        *Connect to camera
        *Run some test to confirm connection
        *Start publishing images on some rostopic
        """


        self.publish_images()

        self.setup_subscribers()

        return


    def setup_subscribers(self):

        # Initializes rosbridge
        self.server = botXimport('rosbridge_api')['rosbridge_suit_component']['module']()
        self.server.setup()

        # intialize buffers
        color_buf = []
        depth_buf = []
        info_buf = []

        # Uses rosbridge to subscribe to camera topics and has a callback to store them in a buffer
        self.server.subscribe(topic='/camera/color/image_raw', type='sensor_msgs/Image', callback=self.cache_color_bridge)
        self.server.subscribe(topic='/camera/depth/image_rect_raw', type='sensor_msgs/Image', callback=self.cache_depth_bridge)
        self.server.subscribe(topic='/camera/color/camera_info', type='sensor_msgs/CameraInfo', callback=self.cache_info_bridge)
        
        return


    def calibrate(self):
        """
        *run specfic function/sequence to calibrate camera
        """
        pass

    def publish_images(self):
        """
        *publish images to some rostopic
        """
        command = 'roslaunch realsense2_camera rs_camera.launch'
        self.publish_proc_id = external_command_pool.start_command(command)
        return

    def cache_color_bridge(self, msg):
        # initialize json color image buffer for bridge
        if not hasattr(self, 'color_buf'):
            self.color_buf = []

        """
        Callback to store stuff read from the bridge into a buffer
        """
        if (len(self.color_buf) > 2000):
            self.color_buf.pop(0)
        self.color_buf.append([msg])
        return

    def cache_depth_bridge(self, msg):
        # initialize json depth image buffer for bridge
        if not hasattr(self, 'depth_buf'):
            self.depth_buf = []

        """
        Callback to store stuff read from the bridge into a buffer
        """
        if (len(self.depth_buf) > 2000):
            self.depth_buf.pop(0)
        self.depth_buf.append([msg])
        return

    def cache_info_bridge(self, msg):

        if not hasattr(self, 'info_buf'):
            self.info_buf = []

        """
        Callback to store stuff read from the bridge into a buffer
        """
        if (len(self.info_buf) > 2000):
            self.info_buf.pop(0)
        self.info_buf.append([msg])
        return


    def get_color_image(self):
        """
        *grab and return a color image
        """
        if not hasattr(self, 'color_buf'):
            self.color_buf = []

        while not self.color_buf:
            time.sleep(1)
        image = [x[0] for x in self.color_buf]
        im = image[-1:]
        return im[0]

    def get_depth_image(self):
        
        """
        *grap and return a depth image
        """
        if not hasattr(self, 'depth_buf'):
            self.depth_buf = []

        while not self.depth_buf:
            time.sleep(1)
        image = [x[0] for x in self.depth_buf]
        im = image[-1:]
        return im[0]

    def shutdown(self):
        external_command_pool.end_command(self.publish_proc_id)
        pass

