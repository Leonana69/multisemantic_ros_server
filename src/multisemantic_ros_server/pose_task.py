import rospy
import numpy as np
from multisemantic_ros_server.srv import MSCVPose

class PoseTask():
    def __init__(self):
        rospy.wait_for_service('mscv_pose_service')

    def request(self, image_msg):
        try:
            mscv_pose = rospy.ServiceProxy('mscv_pose_service', MSCVPose)
            resp = mscv_pose(image_msg)
            print('resp: ', resp.keypoints)
            return np.array(resp.keypoints).reshape((17, 3))
        except rospy.ServiceException as e:
            print("[PT] mscv_pose_service call failed: %s"%e)
            return []