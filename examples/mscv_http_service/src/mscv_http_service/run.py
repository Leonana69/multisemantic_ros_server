#!/usr/bin/env python3
import rospy
import threading

from multisemantic_ros_server.http_server import main

if __name__ == '__main__':
	threading.Thread(target=lambda: rospy.init_node('mscv_server', disable_signals=True)).start()
	main()
