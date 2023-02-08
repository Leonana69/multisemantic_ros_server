import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

def slam_callback(data, slam_task):
	print('get one pose')
	slam_task.pose.append(data)

class SLAMTask():
	def __init__(self):
		self.pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
		self.sub = rospy.Subscriber('/orb_pose', PoseStamped, slam_callback, self)
		self.pose = []

	def request(self, image_msg):
		if not rospy.is_shutdown():
			self.pub.publish(image_msg)
		else:
			print('[SL] ros is down')
	
	def collect(self):
		pose = self.pose
		if len(pose) > 0:
			print(type(pose[0]), len(pose))
		self.pose = []
		return pose
