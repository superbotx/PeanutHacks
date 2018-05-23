from botX.components import BaseComponent
from botX.applications import external_command_pool, botXimport

class CameraAPI(BaseComponent):

	def setup():
		"""
		*Connect to camera
		*Run some test to confirm connection
		*Start publishing images on some rostopic
		"""
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
		pass

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
		pass

