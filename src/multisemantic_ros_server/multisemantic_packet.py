import json

class MultisemanticPacket():
    mode = ['single_image', 'stream', 'stop']
    function = ['pose', 'slam']
    image_format = ['none', 'raw', 'cv_compressed', 'ros_msg']
    users_list = ['duke_drone_1', 'duke_drone_2', 'web_interface', 'guojun']

    def __init__(self, user='', mode='', timestamp=0.0, function=[], msg=[], image=None, imu=None):
        self.user = user
        self.mode = mode
        self.timestamp = timestamp
        self.function = function
        self.msg = msg
        self.image = image
        self.imu = imu
        self.result = []

    def from_json_str(str_packet):
        # parse client packet
        try:
            user = ''
            mode = ''
            timestamp = 0.0
            function = []
            msg = []
            image = None
            imu = None
            json_packet = json.loads(str_packet)
        except ValueError as e:
            print('[MP] parse json [FAILED]')
            return MultisemanticPacket()

        if 'user' in json_packet:
            user = json_packet['user']

        if 'mode' in json_packet:
            mode = json_packet['mode']

        if 'timestamp' in json_packet:
            timestamp = json_packet['timestamp']

        if 'function' in json_packet and type(json_packet['function']) is list:
            function = json_packet['function']

        if 'image' in json_packet:
            image = json_packet['image']

        if 'imu' in json_packet:
            imu = json_packet['imu']

        return MultisemanticPacket(user, mode, timestamp, function, msg, image, imu)

    def is_valid(self):
        is_valid = True
        if len(self.user) == 0 or self.user not in MultisemanticPacket.users_list:
            self.msg.append('[ERROR] invliad user identification')
            is_valid = False

        if len(self.mode) == 0:
            is_valid = False
            self.msg.append(f'[ERROR] mode field is empty, the valid values are: {MultisemanticPacket.mode}')
        elif self.mode not in MultisemanticPacket.mode:
            is_valid = False
            self.msg.append(f'[ERROR] invalid mode: {self.mode}, the valid values are: {MultisemanticPacket.mode}')

        if self.timestamp == 0.0:
            self.msg.append(f'[WARN] no timestamp')

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

        # this should be consistant with function
        if self.image is None:
            is_valid = False
            self.msg.append(f'[ERROR] empty image field')
        elif 'format' not in self.image or 'data' not in self.image:
            is_valid = False
            self.msg.append(f'[ERROR] invalid image field (\'format\' and \'data\' are required)')

        return is_valid

    def get_server_packet(self):
        r_packet = {
            'user': self.user,
            'mode': self.mode,
            'timestamp': self.timestamp,
            'function': self.function,
            'msg': self.msg,
            'result': self.result,
        }
        return json.dumps(r_packet)
