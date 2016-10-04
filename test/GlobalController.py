# from detect_aruco_image import *
from detect_aruco_flycam import *
import time
import math
import sys
import os
import numpy as np
import cgbot
from cgbot.commands import cmd
import subprocess
from imports import WrapperForPointGray

BOT_SPEED = 4000
CUTOFF_DISTANCE = 1.3
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


# rotate bot (sharply) in given direction
def rotateAndMove(direction):
    # turn by TURN_AMT for TURN_TIME
    TURN_AMT = 0.7
    TURN_TIME = 0.5
    if direction:
        cmd.turn(TURN_AMT)
        cmd.turn(TURN_AMT)
        cmd.turn(TURN_AMT)
    else:
        cmd.turn(-TURN_AMT)
        cmd.turn(-TURN_AMT)
        cmd.turn(-TURN_AMT)
    time.sleep(TURN_TIME)
    # cmd.forward(speed=BOT_SPEED)


# move bot from current position, orientation towards given target
def moveTowardsTarget(Rvec, Tvec):
    # Get the orientation of the object using Rvec and Tvec
    R = cv2.Rodrigues(Rvec)
    euler_angles = rotationMatrixToEulerAngles(R[0])
    z_angle = euler_angles[1]
    print "Z-angle: ", z_angle

    # Move towards the object for CALIBRATION_SLEEP_TIME
    CALIBRATION_TURN_AMT = 0.3
    CALIBRATION_SLEEP_TIME = 0.3
    if(z_angle < -THRESHOLD_ANGLE):
        print "Calibration-> Right Turn"
        cmd.turn(CALIBRATION_TURN_AMT)
        cmd.turn(CALIBRATION_TURN_AMT)
        cmd.turn(CALIBRATION_TURN_AMT)
        time.sleep(CALIBRATION_SLEEP_TIME)
    elif(z_angle > THRESHOLD_ANGLE):
        print "Calibration-> Left Turn"
        cmd.turn(-CALIBRATION_TURN_AMT)
        cmd.turn(-CALIBRATION_TURN_AMT)
        cmd.turn(-CALIBRATION_TURN_AMT)
        time.sleep(CALIBRATION_SLEEP_TIME)
    cmd.forward(speed=BOT_SPEED)
    return


turnMode = False
calibrateMode = True


def main():
    global calibrateMode
    global turnMode

    cmd.forward(speed=BOT_SPEED)
    time.sleep(SLEEP_TIME)

    undetectedIterations = 0
    turnState = 0
    turnId = -1
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

            # # Draw marker on observedframe image
            # marker.draw(img, np.array([255, 0, 0]), 10, True)

            print "Id:", marker.id
            # print "Rvec:\n", marker.Rvec
            # print "Tvec:\n", marker.Tvec
            print "Distance:\n", marker.Tvec[2][0]

            if calibrateMode:
                # ---------------- CALIBRATE MODE ---------------
                # take action according to distance and id of detected marker
                marker_distance = marker.Tvec[2][0]
                if(marker_distance < CUTOFF_DISTANCE):
                    print "******************"
                    print "Detected threshold"
                    print "******************"
                    calibrateMode = False
                    turnMode = True
                else:
                    moveTowardsTarget(marker.Rvec, marker.Tvec)

            if turnMode:
                # ---------------- TURN MODE ---------------
                turnId = marker.id
                while True:
                    rotateAndMove(bool(turnState))
                    markers = getMarkersFromCurrentFrame()
                    for marker in markers:
                        if(not(marker.id == turnId)):
                            break
                turnMode = False
                calibrateMode = True
                cmd.forward(speed=BOT_SPEED)
                # Change turnstate for next turn
                turnState = turnState ^ 1
                print "Changed turnstate to: ", turnState

        else:
            undetectedIterations = undetectedIterations + 1

        if(undetectedIterations > 100):
            cmd.stop()

        # sleep
        time.sleep(SLEEP_TIME)


if __name__ == '__main__':
    init()
    main()
