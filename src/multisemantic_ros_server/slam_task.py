import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

class SLAMTask():
	def __init__(self):
		self.pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
		self.sub = rospy.Subscriber('/orb_pose', geometry_msgs::PoseStamped, SLAMTask::slam_callback)
		self.pose = []

	def request(self, image_msg):
		if not rospy.is_shutdown():
			self.pub.publish(image_msg)
		else:
			print('[SL] ros is down')
	
	def slam_callback(data):
		print('get one pose')
		# slam_task.pose.append(data)
		print('append one pose')
