import json
import cv2
import numpy as np

class MultisemanticPacket():
    mode = ['single_image', 'stream']
    function = ['pose', 'slam']

    def __init__(self, user='', mode='', function=[], msg=[], image=None):
        self.user = user
        self.mode = mode
        self.function = function
        self.msg = msg
        self.image = image
        self.result = []

    def from_json_str(str_packet):
        # parse client packet
        try:
            user = ''
            mode = ''
            function = []
            msg = []
            image = None
            json_packet = json.loads(str_packet)
        except ValueError as e:
            print('[MP] parse json [FAILED]')
            return MultisemanticPacket()

        if 'user' in json_packet:
            user = json_packet['user']
            
        if 'mode' in json_packet:
            mode = json_packet['mode']

        if 'function' in json_packet and type(json_packet['function']) is list:
            function = json_packet['function']

        if 'image' in json_packet:
            img = np.array(json_packet['image'], dtype=np.uint8)
            image = cv2.imdecode(img, cv2.IMREAD_COLOR)

        return MultisemanticPacket(user, mode, function, msg, image)

    def is_valid(self):
        is_valid = True
        if len(self.user) == 0:
            self.msg.append('[WARNING] no user identification')

        if len(self.mode) == 0:
            is_valid = False
            self.msg.append(f'[ERROR] mode field is empty, the valid values are: {MultisemanticPacket.mode}')
        elif self.mode not in MultisemanticPacket.mode:
            is_valid = False
            self.msg.append(f'[ERROR] invalid mode: {self.mode}, the valid values are: {MultisemanticPacket.mode}')

        if len(self.function) == 0:
            is_valid = False
            self.msg.append(f'[ERROR] function field is empty, the valid values are: {MultisemanticPacket.function}')
        else:
            valid_function = []
            for f in self.function:
                if f not in MultisemanticPacket.function:
                    self.msg.append(f'[WARNING] invalid function: {f}, the valid values are: {MultisemanticPacket.function}')
                else:
                    valid_function.append(f)
            self.function = valid_function
            if len(self.function) == 0:
                is_valid = False
                self.msg.append('[ERROR] no valid function')

        if self.image is None:
            is_valid = False
            self.msg.append(f'[ERROR] empty image field')
        elif not self.image.any():
            is_valid = False
            self.msg.append(f'[ERROR] image decode failed')

        return is_valid

    def get_server_packet(self):
        r_packet = {
            'user': self.user,
            'mode': self.mode,
            'function': self.function,
            'msg': self.msg,
            'result': self.result,
        }
        return json.dumps(r_packet)
