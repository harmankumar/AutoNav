# from detect_aruco_image import *
#from detect_aruco_flycam import *
import time
import math
import sys
import os
import numpy as np
import cgbot
from cgbot.commands import cmd
import subprocess
from imports import WrapperForPointGray
from EncoderReading import *
from cgbot.redisdb import rdb
from cgbot.robot import rbt
from cgbot.detect import motorcontroller
from cgbot.sensors import Orientation
import control_servo as cs
import MobileCommunication as mc
#cs.rotate(angle)


def orient():
    compass = Orientation()
    print compass.yaw
    sys.exit(0)

BOT_SPEED = 4000
CUTOFF_DISTANCE = 1.0
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
    print "Turning"
    TURN_AMT = 0.4
    TURN_TIME = 0.3
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


calibrate_angle_list = []
calibrationDirection = 1

# move bot from current position, orientation towards given target
def moveTowardsTarget(Rvec, Tvec):
    # Move towards the object for CALIBRATION_SLEEP_TIME
    CALIBRATION_TURN_AMT = 0.3
    CALIBRATION_SLEEP_TIME = 0.3
    # Get the orientation of the object using Rvec and Tvec
    R = cv2.Rodrigues(Rvec)
    euler_angles = rotationMatrixToEulerAngles(R[0])
    z_angle = euler_angles[1]
    print "Z-angle: ", z_angle
    calibrate_angle_list.append(z_angle)

    # Find sign of z_angle
    l = len(calibrate_angle_list)
    if(l < 7):
        return
    global calibrationDirection
    if(calibrate_angle_list[l - 1] > calibrate_angle_list[l - 7]):
        calibrationDirection = - calibrationDirection
    cmd.turn(calibrationDirection * CALIBRATION_TURN_AMT)
    cmd.turn(calibrationDirection * CALIBRATION_TURN_AMT)
    cmd.turn(calibrationDirection * CALIBRATION_TURN_AMT)
    time.sleep(CALIBRATION_SLEEP_TIME)

    # if(z_angle < -THRESHOLD_ANGLE):
    #     print "Calibration-> Right Turn"
    #     cmd.turn(CALIBRATION_TURN_AMT)
    #     cmd.turn(CALIBRATION_TURN_AMT)
    #     cmd.turn(CALIBRATION_TURN_AMT)
    #     time.sleep(CALIBRATION_SLEEP_TIME)
    # elif(z_angle > THRESHOLD_ANGLE):
    #     print "Calibration-> Left Turn"
    #     cmd.turn(-CALIBRATION_TURN_AMT)
    #     cmd.turn(-CALIBRATION_TURN_AMT)
    #     cmd.turn(-CALIBRATION_TURN_AMT)
    #     time.sleep(CALIBRATION_SLEEP_TIME)
    cmd.forward(speed=BOT_SPEED)
    time.sleep(CALIBRATION_SLEEP_TIME)
    return

def readMotrTicks():
    temp = rbt.readTick()
    return (temp[0][1],temp[1][1])



turnMode = False
calibrateMode = True


def main():
    global calibrateMode
    global turnMode
    rbt.connect(motorcontroller())

    mc.initsocket()
    print mc.getYaw()


    exit(0)

    #cs.write_servo(0)

    undetectedIterations = 0
    turnState = 0
    turnId = -1
    doneList = set()
    prev_id = -1

    # # Get rid of initial lag
    # print "Initial"
    # (frame, markers) = getMarkersFromCurrentFrame()
    # if len(markers) > 0:
    #     markers[0].draw(frame, np.array([255, 0, 0]), 10, True)
    # cv2.imshow("frame", frame)
    # cv2.waitKey(10)
    # time.sleep(5)

    print "Start bot"
    cmd.forward(speed=BOT_SPEED)
    time.sleep(SLEEP_TIME)

    while(True):
        print "Start loop"
        # get an image
        # markers = findImageArucoParams('ar2.png')
        (frame, markers) = getMarkersFromCurrentFrame()

        # Process markers
        print "Markers detected", len(markers)
        if len(markers) > 0:
            undetectedIterations = 0
            marker = markers[0]
            for m in markers:
                if not(m in doneList):
                    marker = m
            marker.calculateExtrinsics(marker_size, camparam)
            # min_d = markers[0].Tvec[2][0]
            # marker = markers[0]
            # for m in markers:
            #     m.calculateExtrinsics(marker_size, camparam)
            #     current_distance = m.Tvec[2][0]
            #     if (current_distance < min_d):
            #         min_d = current_distanceexit(0)

            #         marker = m

            # # Draw marker on observedframe image
            # marker.draw(frame, np.array([255, 0, 0]), 10, True)

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
                # turnId = marker.id
                while True:
                    toBreak = False
                    rotateAndMove(bool(turnState))
                    (frame, markers) = getMarkersFromCurrentFrame()

                    global calibrate_angle_list
                    calibrate_angle_list = []
                    doneList.add(marker.id)
                    print "Added to doneList: ", marker.id
                    print doneList
                    if len(markers) > 0:
                        for marker_curr in markers:
                            print "Seeing current marker_curr"
                            marker_curr.calculateExtrinsics(marker_size, camparam)
                            R = cv2.Rodrigues(marker_curr.Rvec)
                            euler_angles = rotationMatrixToEulerAngles(R[0])
                            z_angle = euler_angles[1]
                            if(not(marker_curr.id in doneList) and z_angle < 0.15):
                                print "Detected new marker"
                                rotateAndMove(bool(turnState))
                                toBreak = True
                                break
                    else :
                        print "Undetected", undetectedIterations
                        undetectedIterations = undetectedIterations + 1
                        if(undetectedIterations > 30):
                            cmd.stop()
                    if(toBreak):
                        break
                turnMode = False
                calibrateMode = True
                print "Exited turn mode"
                cmd.forward(speed=BOT_SPEED)
                # Change turnstate for next turn
                turnState = turnState ^ 1
                print "Changed turnstate to: ", turnState

        else:
            print "Undetected", undetectedIterations
            undetectedIterations = undetectedIterations + 1

        if(undetectedIterations > 20):
            cmd.stop()

        # # show frame
        # cv2.imshow("frame", frame)
        # cv2.waitKey(int(SLEEP_TIME * 1000))

        time.sleep(SLEEP_TIME)


if __name__ == '__main__':
    #orient()
    #init()
    main()
