import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class SLAMTask():
	def __init__(self):
		self.bridge = CvBridge()
		self.pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)

	def run(self, image):
		if not rospy.is_shutdown():
			msg = self.bridge.cv2_to_imgmsg(image)
			self.pub.publish(msg)
		else:
			print('[SL] ros is down')