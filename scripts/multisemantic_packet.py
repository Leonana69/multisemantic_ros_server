import json
import cv2
import numpy as np

class MultisemanticPacket():
    mode = ['single_image', 'stream']
    function = ['pose', 'slam', 'face', 'hands']

    def __init__(self, str_packet):
        # parse client packet
        try:
            self.is_valid = True
            self.msg = []
            self.result = []
            self.function = []
            self.mode = 'None'

            json_packet = json.loads(str_packet)
        except ValueError as e:
            print('[MP] parse json [FAILED]')
            self.is_valid = False
            return

        if 'user' in json_packet:
            self.user = json_packet['user']
        else:
            self.user = 'default_user'
            self.msg.append('[INIT] user field is emtpy, use default_user')

        if 'mode' in json_packet:
            m = json_packet['mode']
            if m in MultisemanticPacket.mode:
                self.mode = json_packet['mode']
            else:
                self.msg.append('[INIT] invalid mode: {}'.format(m))
        else:
            self.mode = 'single_image'
            self.msg.append('[INIT] mode field is emtpy, use single_image')


        if 'function' in json_packet and type(json_packet['function']) is list:
            for f in json_packet['function']:
                if f in MultisemanticPacket.function:
                    self.function.append(f)
                else:
                    self.msg.append('[INIT] invalid function: {}'.format(f))
        else:
            self.msg.append('[INIT] no function is assigned')

        if 'image' in json_packet:
            img = np.array(json_packet['image'], dtype=np.uint8)
            self.image = cv2.imdecode(img, cv2.IMREAD_COLOR)
            if not self.image.any():
                print('[MP] image decode [FAILED]')
                self.msg.append('[INIT] image decode failed')
        else:
            print('[MP] no image')
            self.msg.append('[INIT] image field is empty')
            self.is_valid = False

    def get_server_packet(self):
        r_packet = {
            'user': self.user,
            'mode': self.mode,
            'function': self.function,
            'msg': self.msg,
            'result': self.result,
        }
        return json.dumps(r_packet)
