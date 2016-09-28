import cv2
import numpy as np
import aruco

if __name__ == '__main__':
    # create markerDetector object
    markerdetector = aruco.MarkerDetector()
    camparam = aruco.CameraParameters()

    # Load an color image in grayscale
    img = cv2.imread('marker1.jpg', cv2.IMREAD_COLOR)

    markers = markerdetector.detect(img, camparam)
    print "Markers detected", len(markers)
    for marker in markers:
        marker.draw(img, np.array([255, 0, 0]), 100, True)
        for i, point in enumerate(marker):
            print i, point

    cv2.namedWindow('detected', cv2.WINDOW_NORMAL)
    cv2.imshow('detected', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
