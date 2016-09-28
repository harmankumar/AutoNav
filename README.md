Dependencies:
aruco (http://www.uco.es/investiga/grupos/ava/node/26)
python-aruco (https://github.com/fehlfarbe/python-aruco)
swig (dependency of python-aruco)

Running:
test/ directory contains code for calibration of camera, detection of aruco markers from image and video.
calibrate.py -> camera calibration
common.py -> requirement of calibrate.py
cam_parameters.yml -> camera parameters for aruco detection in image/video
detect_aruco_image.py -> detect aruco marker in image
detect_aruco_video.py -> detect aruco marker in video
