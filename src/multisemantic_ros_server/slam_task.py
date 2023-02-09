import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

def slam_callback(data, slam_task):
	slam_task.pose.append(str(data))

class SLAMTask():
	def __init__(self):
		self.pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
		self.sub = rospy.Subscriber('/orb_pose', PoseStamped, slam_callback, self)
		self.pose = []

	def request(self, image_msg):
		self.pub.publish(image_msg)
		ret = self.collect()
		if len(ret) > 0:
			return self.collect(), '[SL] request slam [SUCCESS]'
		else:
			return [], '[SL] no results'
	
	def collect(self):
		pose = self.pose
		self.pose = []
		return pose
