# from detect_aruco_image import *
from detect_aruco_flycam import *
import time
import math
import sys
import os
import cgbot
from cgbot.commands import cmd
import subprocess
from imports import WrapperForPointGray

BOT_SPEED = 5000
CUTOFF_DISTANCE = 1.5
THRESHOLD_ANGLE = 0.1  # in radians
SLEEP_TIME = 0.05


def init():
    process1 = subprocess.call('cd /home/piyush/run-bot', shell=True)
    # process1 = Popen(['cd', '/home/piyush/run-bot'])
    process2 = subprocess.Popen(['/home/piyush/run-bot/run1.sh > /dev/null'], shell=True)
    process3 = subprocess.Popen(['/home/piyush/run-bot/run2.sh > /dev/null'], shell=True)
    time.sleep(3)
    process4 = subprocess.call('cd /home/piyush/OpenCV3_Local/test', shell=True)


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


# modes
pursueMode = False
calibrateMode = False


def main():

    cmd.forward(speed=BOT_SPEED)
    time.sleep(SLEEP_TIME)

    undetectedIterations = 0
    turnState = 0
    prev_id = -1

    while(True):

        # get an image
        # markers = findImageArucoParams('ar2.png')
        markers = getMarkersFromCurrentFrame()

        # Process markers
        print "Markers detected", len(markers)
        if len(markers) > 0:
            undetectedIterations = 0
            min_d = markers[0].Tvec[2][0]
            marker = markers[0]
            for m in markers:
                m.calculateExtrinsics(marker_size, camparam)
                current_distance = m.Tvec[2][0]
                if (current_distance < min_d):
                    min_d = current_distance
                    marker = m

            # marker.draw(img, np.array([255, 0, 0]), 100, True)
            # for i, point in enumerate(marker):
            #     print i, point

            print "Id:", marker.id
            # print "Rvec:\n", marker.Rvec
            # print "Tvec:\n", marker.Tvec
            print "Distance:\n", marker.Tvec[2][0]

            if(not(marker.id == prev_id)):
                prev_id = marker.id
                turnState = turnState ^ 1

            # make an action according to detected marker.
            if(marker.Tvec[2][0] < CUTOFF_DISTANCE and turnState == 0):
                cmd.turn(0.9)
                time.sleep(3)
            if(marker.Tvec[2][0] < CUTOFF_DISTANCE and turnState == 1):
                cmd.turn(-0.9)
                time.sleep(3)

            # find angle with z axis
            R = cv2.Rodrigues(marker.Rvec)
            euler_angles = rotationMatrixToEulerAngles(R[0])
            z_angle = euler_angles[1]

            if(z_angle < -THRESHOLD_ANGLE):
                cmd.turn(0.8)
                time.sleep(.2)
            elif(z_angle > THRESHOLD_ANGLE):
                cmd.turn(-0.8)
                time.sleep(.2)
            else:
                cmd.forward()
        else:
            undetectedIterations = undetectedIterations + 1

        if(undetectedIterations > 100):
            cmd.stop()

        # sleep
        time.sleep(SLEEP_TIME)


if __name__ == '__main__':
    init()
    main()
