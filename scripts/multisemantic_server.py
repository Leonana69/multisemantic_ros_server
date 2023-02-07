from slam_task import SLAMTask
# from pose_task import PoseTask

class MultisemanticServer():
    def __init__(self):
        self.slam_task = SLAMTask()
        # self.pose_task = PoseTask()

    def run(self, m_packet):
        result = []
        for f in m_packet.function:
            entry = {
                'function': f,
                'output': ''
            }

            if f == 'pose':
                print('run pose function1')
                # entry['output'] = self.pose_task.run(m_packet.image)
                print('run pose function2')
            elif f == 'slam':
                self.slam_task.run(m_packet.image)
            else:
                print('undefined function')
                continue

            result.append(entry)
        m_packet.result = result