import math
import cv2
import numpy as np
import aruco


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])


if __name__ == '__main__':
    # create markerDetector object
    # f = open("Mat.txt", 'a')

    markerdetector = aruco.MarkerDetector()
    camparam = aruco.CameraParameters()
    camparam.readFromXMLFile("cam_params_flycap.yml")\

    marker_size = 0.165

    # Load an color image in grayscale
    img = cv2.imread('5.png', cv2.IMREAD_COLOR)

    markers = markerdetector.detect(img, camparam)
    print "Markers detected", len(markers)
    for marker in markers:
        marker.draw(img, np.array([255, 0, 0]), 10, True)
        # for i, point in enumerate(marker):
        #     print i, point

        marker.calculateExtrinsics(marker_size, camparam)
        # print "Id:", marker.id
        # print "Rvec:\n", marker.Rvec
        # print "Tvec:\n", marker.Tvec
        # print "Rotation Matrix: "
        R = cv2.Rodrigues(marker.Rvec)
        euler_angles = rotationMatrixToEulerAngles(R[0])
        z_angle = euler_angles[1]
        print euler_angles
        # np.savetxt(f, cv2.Rodrigues(marker.Rvec)[0], delimiter=',')

        # f.write('\n\n')
    # f.close()

    cv2.namedWindow('detected', cv2.WINDOW_NORMAL)
    cv2.imshow('detected', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
