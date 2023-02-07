import cv2

def draw_pose_keypoints(image, keypoints):
    output = image.copy()
    height = output.shape[0]
    width = output.shape[1]
    for p in keypoints:
        cv2.circle(output, (int(width * p[1]), int(height * p[0])), 2, (255, 0, 0), 2)
    return output