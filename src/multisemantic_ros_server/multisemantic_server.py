from multisemantic_ros_server.slam_task import SLAMTask
from multisemantic_ros_server.pose_task import PoseTask
from cv_bridge import CvBridge

class MultisemanticServer():
    def __init__(self):
        self.bridge = CvBridge()
        self.slam_task = SLAMTask()
        self.pose_task = PoseTask()

    def run(self, m_packet):
        result = []
        image_msg = self.bridge.cv2_to_imgmsg(m_packet.image)
        for f in m_packet.function:
            entry = {
                'function': f,
                'output': ''
            }

            if f == 'pose':
                entry['output'] = self.pose_task.request(image_msg)
            elif f == 'slam':
                self.slam_task.request(image_msg)
                entry['output'] = self.slam_task.collect()
            else:
                print('undefined function')
                continue

            result.append(entry)
        m_packet.result = result
