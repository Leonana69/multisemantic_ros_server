#!/usr/bin/env python3
import rospy

from multisemantic_ros_server.http_server import main
from multisemantic_ros_server import http_server

app = Flask(__name__)
app.config['UPLOAD_IMAGE_PATH'] = 'assets/images/'
app.config['OUTPUT_IMAGE_PATH'] = 'assets/outputs/'
app.config['MAX_CONTENT_LENGTH'] = 16 * 1024 * 1024
app.config['ALLOWED_EXTENSIONS'] = ['.jpg', '.jpeg', '.png']
app.config['ALLOWED_FUNCTIONS'] = ['pose', 'slam', 'hands', 'face']

app.add_url_rule('/', view_func=http_server.index)
app.add_url_rule('/upload', methods=['POST'], view_func=http_server.upload)
app.add_url_rule('//serve-image/<filename>', methods=['GET'], view_func=http_server.serve_image)
app.add_url_rule('/api', methods=['POST'], view_func=http_server.json_api)

if __name__ == '__main__':
	rospy.init_node('mscv_server', anonymous=False, disable_signals=True)
	app.run(host='localhost', port=50001)