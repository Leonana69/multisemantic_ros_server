import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

def slam_callback(data, slam_task):
	slam_task.pose.append(str(data))

class SLAMTask():
	def __init__(self, user):
		self.pub = rospy.Publisher('/camera/image_' + user, Image, queue_size=1)
		self.sub = rospy.Subscriber('/slam_res_' + user, PoseStamped, slam_callback, self)
		self.pose = []

	def request(self, image_msg):
		self.pub.publish(image_msg)
		ret = self.collect()
		if len(ret) > 0:
			return ret, '[SL] request slam [SUCCESS]'
		else:
			return [], '[SL] no results'
	
	def collect(self):
		pose = self.pose
		self.pose = []
		return pose
