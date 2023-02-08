#!/usr/bin/env python3

import rospy
from multisemantic_ros_server.cv.pose_tf import PoseTF
from multisemantic_ros_server.srv import MSCVPose, MSCVPoseResponse
from cv_bridge import CvBridge
bridge = CvBridge()
interpreter = PoseTF()

def pose(req):
    print('[P] get a request')
    keypoints = interpreter.run(bridge.imgmsg_to_cv2(req.img))
    return MSCVPoseResponse(keypoints.ravel())

def pose_server():
    rospy.init_node('mscv_pose_server')
    s = rospy.Service('mscv_pose_service', MSCVPose, pose)
    print("[P] Ready to serve.")
    rospy.spin()

if __name__ == '__main__':
    pose_server()