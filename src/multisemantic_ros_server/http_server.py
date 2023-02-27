#!/usr/bin/env python3
from flask import Flask, render_template, request, send_from_directory
from werkzeug.utils import secure_filename
from werkzeug.exceptions import RequestEntityTooLarge
import os, cv2, json
import numpy as np
from multisemantic_ros_server.utils import draw_pose_keypoints
from multisemantic_ros_server.multisemantic_packet import MultisemanticPacket
from multisemantic_ros_server.multisemantic_server import MultisemanticServer

basedir = os.path.abspath(os.path.dirname(__file__))

app = Flask(__name__)
app.config['UPLOAD_IMAGE_PATH'] = os.path.join(basedir, 'assets/images/')
app.config['OUTPUT_IMAGE_PATH'] = os.path.join(basedir, 'assets/outputs/')
app.config['MAX_CONTENT_LENGTH'] = 16 * 1024 * 1024
app.config['ALLOWED_EXTENSIONS'] = ['.jpg', '.jpeg', '.png']

multisemantic_handle = MultisemanticServer()

@app.route('/')
def index():

    # print(app.config['OUTPUT_IMAGE_PATH'])
    # files = os.listdir(app.config['OUTPUT_IMAGE_PATH'])
    # images = []
    # for file in files:
    #     extension = os.path.splitext(file)[1]
    #     if extension in app.config['ALLOWED_EXTENSIONS']:
    #         images.append(file)
    # return render_template('index.html', images=images)
    return render_template('index.html')

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
            image = {
                'format': 'raw',
                'data': cv2.imdecode(np.fromstring(image_str, np.uint8), cv2.IMREAD_COLOR)
            }
            m_packet = MultisemanticPacket('web_interface', 'single_image', 0.0, function.split(','), [], image)
            if m_packet.is_valid():
                multisemantic_handle.run(m_packet)

            marked_image = draw_pose_keypoints(image['data'], np.array(m_packet.result[0]['output']))
            cv2.imwrite(os.path.join(app.config['OUTPUT_IMAGE_PATH'], secure_filename(file.filename)), marked_image)
            cv2.imwrite(os.path.join(app.config['UPLOAD_IMAGE_PATH'], secure_filename(file.filename)), image['data'])

    except RequestEntityTooLarge:
        return 'File exceeds the 16MB limit.'

    result_list = [secure_filename(file.filename)]
    return render_template('index.html', images=result_list, keypoints=json.dumps(m_packet.result))

@app.route('/serve-image/<filename>', methods=['GET'])
def serve_image(filename):
    return send_from_directory(app.config['OUTPUT_IMAGE_PATH'], filename)

@app.route('/api', methods=['POST'])
def json_api():
    m_packet = MultisemanticPacket.from_json_str(request.data)
    if m_packet.is_valid():
        multisemantic_handle.run(m_packet)

    return m_packet.get_server_packet()

def main():
    app.run(host='0.0.0.0', port=50001)
