from multisemantic_ros_server.slam_task import SLAMTask
from multisemantic_ros_server.pose_task import PoseTask
from multisemantic_ros_server.multisemantic_packet import MultisemanticPacket
from cv_bridge import CvBridge
import cv2
import numpy as np

class MultisemanticServer():
    def __init__(self):
        self.bridge = CvBridge()
        # self.slam_task = SLAMTask()
        self.slam_nodes = {}
        self.pose_task = PoseTask()

    def run(self, m_packet):
        result = []

        if m_packet.image['format'] == MultisemanticPacket.image_format[1]:
            image_msg = self.bridge.cv2_to_imgmsg(m_packet.image['data'])
        elif m_packet.image['format'] == MultisemanticPacket.image_format[2]:
            img = np.array(m_packet.image['data'], dtype=np.uint8)
            image = cv2.imdecode(img, cv2.IMREAD_COLOR)
            image_msg = self.bridge.cv2_to_imgmsg(image)
        elif m_packet.image['format'] == MultisemanticPacket.image_format[3]:
            image_msg = m_packet.image['data']
        else:
            image_msg = None

        for f in m_packet.function:
            entry = {
                'function': f,
                'output': ''
            }

            if f == 'pose':
                entry['output'], msg = self.pose_task.request(image_msg)
            elif f == 'slam':
                if m_packet.user not in self.slam_nodes:
                    self.slam_nodes[m_packet.user] = SLAMTask(m_packet.user)
                    msg = 'Init SLAM Node'
                elif m_packet.mode == 'stop':
                    del self.slam_nodes[m_packet.user]
                    msg = 'Remove SLAM Node'
                else:
                    entry['output'], msg = self.slam_nodes[m_packet.user].request(image_msg)
            else:
                print('undefined function')
                continue
            
            m_packet.msg.append(msg)
            result.append(entry)
        m_packet.result = result
