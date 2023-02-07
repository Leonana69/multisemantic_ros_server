#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError

from flask import Flask, render_template, request, redirect, send_from_directory
from werkzeug.utils import secure_filename
from werkzeug.exceptions import RequestEntityTooLarge
import os, cv2, json
import numpy as np
from utils import draw_pose_keypoints
from multisemantic_packet import MultisemanticPacket
from multisemantic_server import MultisemanticServer

app = Flask(__name__)
app.config['UPLOAD_IMAGE_PATH'] = 'assets/images/'
app.config['OUTPUT_IMAGE_PATH'] = 'assets/outputs/'
app.config['MAX_CONTENT_LENGTH'] = 16 * 1024 * 1024
app.config['ALLOWED_EXTENSIONS'] = ['.jpg', '.jpeg', '.png']
app.config['ALLOWED_FUNCTIONS'] = ['pose', 'slam', 'hands', 'face']

multisemantic_handle = MultisemanticServer()

@app.route('/')
def index():
    files = os.listdir(app.config['OUTPUT_IMAGE_PATH'])
    images = []
    for file in files:
        extension = os.path.splitext(file)[1]
        if extension in app.config['ALLOWED_EXTENSIONS']:
            images.append(file)
    return render_template('index.html', images=images)

@app.route('/upload', methods=['POST'])
def upload():
    try:
        file = request.files['filename']
        function = request.form['function']
        extension = os.path.splitext(file.filename)[1].lower()

        if file:
            if extension not in app.config['ALLOWED_EXTENSIONS']:
                return 'File is not an image.'
            
            image_str = file.read()
            origin_image = cv2.imdecode(np.fromstring(image_str, np.uint8), cv2.IMREAD_COLOR)
            result = multisemantic_service([function], origin_image)
            marked_image = draw_pose_keypoints(origin_image, np.array(result[0]['output']))
            cv2.imwrite(os.path.join(app.config['OUTPUT_IMAGE_PATH'], secure_filename(file.filename)), marked_image)
            cv2.imwrite(os.path.join(app.config['UPLOAD_IMAGE_PATH'], secure_filename(file.filename)), origin_image)

    except RequestEntityTooLarge:
        return 'File exceeds the 16MB limit.'

    result_list = [secure_filename(file.filename)]
    return render_template('index.html', images=result_list, keypoints=json.dumps(result))

@app.route('/serve-image/<filename>', methods=['GET'])
def serve_image(filename):
    return send_from_directory(app.config['OUTPUT_IMAGE_PATH'], filename)

@app.route('/api', methods=['POST'])
def json_api():
    m_packet = MultisemanticPacket(request.data)
    if m_packet.is_valid:
        multisemantic_handle.run(m_packet)

    return m_packet.get_server_packet()

if __name__ == "__main__":
    rospy.init_node('slam_publisher', anonymous=False, disable_signals=True)
    app.run(host='localhost', port=50001)
