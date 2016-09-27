# AutoNav
Module for navigation of self-driving car using vision.

Dependencies
------------

1. python, opencv, numpy
2. aruco (http://www.uco.es/investiga/grupos/ava/node/26)
3. python-aruco (https://github.com/fehlfarbe/python-aruco)
4. swig (dependency of python-aruco) - generates python wrappers for aruco

Running
------------

`test/` directory contains code for calibration of camera, detection of aruco markers from image and video. <br/>
`calibrate.py` -> camera calibration <br/>
`common.py` -> requirement of calibrate.py <br/>
`cam_parameters.yml` -> camera parameters for aruco detection in image/video <br/>
`detect_aruco_image.py` -> detect aruco marker in image <br/>
`detect_aruco_video.py` -> detect aruco marker in video <br/>
`detect_aruco_flycam.py` (to be run with sudo) -> detect aruco markers in video captured through flycam <br/>
