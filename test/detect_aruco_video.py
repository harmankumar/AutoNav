import sys
import cv2
import numpy as np
import aruco

if __name__ == '__main__':
    # load camera parameters
    camparam = aruco.CameraParameters()
    # camparam.readFromXMLFile("cam_params.yml")

    # create markerDetector object
    markerdetector = aruco.MarkerDetector()
    # markerdetector.setMinMaxSize(0.01)

    # load video
    cap = cv2.VideoCapture('video_harman.mp4')
    ret, frame = cap.read()

    if not ret:
        print "unable to open video!"
        sys.exit(-1)

    id_current = 0
    id_next = 1
    prev_id_list = []
    while ret:
        # print "Processing video"
        markers = markerdetector.detect(frame, camparam)
        if len(markers) > 0:
            # print "Found markers"
            id_list = [m.id for m in markers]
            # print "Ids detected", id_list
            id_max = 0
            for marker in markers:
                if(marker.id > id_max and not(marker.id in prev_id_list)):
                    id_max = marker.id
            id_current = id_max
            id_next = id_max

            for marker in markers:
                if(marker.id == id_current):
                    marker_current = marker
                    marker.draw(frame, np.array([255, 255, 0]), 10, True)
                    for i, point in enumerate(marker):
                        print "Current"
                        print i, point

            # transform_matrix = marker_current.calculateExtrinsics()
            # print transform_matrix

        # show frame
        cv2.imshow("frame", frame)
        cv2.waitKey(10)

        # read next frame
        ret, frame = cap.read()
