from botX.components import BaseComponent
from botX.applications import external_command_pool, botXimport
from botXimport

class CameraAPI(BaseComponent):

	def setup():
		"""
		*Connect to camera
		*Run some test to confirm connection
		*Start publishing images on some rostopic
		"""


		self.publish_images()

		# initialize json buffer for bridge
		self.json_buf = []

		# Initializes rosbridge
		self.server = botXimport('rosbridge_api')['rosbridge_suit_component']['module']()
        self.server.setup()

        # Uses rosbridge to subscribe to camera topics and has a callback to store them in a buffer
        self.server.subscribe(topic='/camera/color/image_raw', type='sensor_msgs/Image', callback=self.cache_info_bridge)
        self.server.subscribe(topic='/camera/depth/image_raw', type='sensor_msgs/Image', callback=self.cache_info_bridge)
        self.server.subscribe(topic='/camera/color/camera_info', type='sensor_msgs/CameraInfo', callback=self.cache_info_bridge)
		
		pass


	def calibrate():
		"""
		*run specfic function/sequence to calibrate camera
		"""
		pass

	def publish_images():
		"""
		*publish images to some rostopic
		"""
		command = 'roslaunch realsense2_camera rs_camera.launch'
        self.publish_proc_id = external_command_pool.start_command(command)
		return

	 def cache_info_bridge(self, msg):
        """
        Callback to store stuff read from the bridge into a buffer
        """
        if (len(self.json_buf) > 2000):
            self.json_buf.pop(0)
        self.json_buf.append([msg])
        return

	def get_color_image():
		"""
		*grab and return a color image
		"""

		pass

	def get_depth_image():
		"""
		*grap and return a depth image
		"""
		pass

	def shutdown():
		external_command_pool.end_command(self.publish_proc_id)
		pass

