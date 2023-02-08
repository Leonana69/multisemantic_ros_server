import rospy
from multisemantic_ros_server.srv import SimpleTest

class PoseTask():
    def __init__(self):
        rospy.wait_for_service('mscv_pose_service')

    def request(self, image):
        try:
            mscv_pose = rospy.ServiceProxy('mscv_pose_service', SimpleTest)
            resp = mscv_pose(100)
            print('resp: ', resp.b)
            return resp.b
        except rospy.ServiceException as e:
            print("[PT] mscv_pose_service call failed: %s"%e)
            return []