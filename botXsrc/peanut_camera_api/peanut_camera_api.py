from botX.components import BaseComponent
from botX.applications import external_command_pool, botXimport


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

        # Uses rosbridge to subscribe to camera topics and has a callback to store them in a buffer
        self.server.subscribe(topic='/camera/color/image_raw', type='sensor_msgs/Image', callback=self.cache_info_bridge)
        self.server.subscribe(topic='/camera/depth/image_raw', type='sensor_msgs/Image', callback=self.cache_info_bridge)
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

    def cache_info_bridge(self, msg):

        print (msg)

        # initialize json buffer for bridge
        # if not self.json_buf:
        #     json_buf = []

        """
        Callback to store stuff read from the bridge into a buffer
        """
        # if (len(self.json_buf) > 2000):
        #     self.json_buf.pop(0)
        # self.json_buf.append([msg])
        # return


    def get_color_image(self):
        """
        *grab and return a color image
        """

        pass

    def get_depth_image(self):
        """
        *grap and return a depth image
        """
        pass

    def shutdown(self):
        external_command_pool.end_command(self.publish_proc_id)
        pass

