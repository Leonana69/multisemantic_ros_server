import rospy
from sensor_msgs.msg import Image

class SLAMTask():
	def __init__(self):
		self.pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)

	def request(self, image_msg):
		if not rospy.is_shutdown():
			self.pub.publish(image_msg)
		else:
			print('[SL] ros is down')