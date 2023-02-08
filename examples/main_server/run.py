#!/usr/bin/env python3
import rospy

from multisemantic_ros_server.http_server import main

if __name__ == '__main__':
	rospy.init_node('mscv_server', anonymous=False, disable_signals=True)
	main()