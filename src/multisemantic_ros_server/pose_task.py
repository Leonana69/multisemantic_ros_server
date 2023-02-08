import rospy
from multisemantic_ros_server.srv import MSCVPose

class PoseTask():
    def __init__(self):
        rospy.wait_for_service('mscv_pose_service')

    def request(self, image):
        try:
            mscv_pose = rospy.ServiceProxy('mscv_pose_service', MSCVPose)
            resp = mscv_pose(image)
            return resp.keypoints
        except rospy.ServiceException as e:
            print("[PT] mscv_pose_service call failed: %s"%e)
            return []