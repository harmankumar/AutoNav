import time
import math
import sys
import os
import numpy as np
import cv2
import pyfly2
import subprocess
from imports import WrapperForPointGray
from odometry import *
import lcm
from exlcm import example_t


# cs.rotate(angle)

def orient():
    compass = Orientation()
    # print compass.yaw
    sys.exit(0)

BOT_SPEED = 4000
CUTOFF_DISTANCE = 1.0
THRESHOLD_ANGLE = 0.12  # in radians
SLEEP_TIME = 0.05
PROCESSING_INTERVAL = 0.4

turnAngle = 0
distanceToObstacle = 0


def init():
    process1 = subprocess.call('cd /home/piyush/run-bot', shell=True)
    # process1 = Popen(['cd', '/home/piyush/run-bot'])
    process2 = subprocess.Popen(['/home/piyush/run-bot/run1.sh > /dev/null'], shell=True)
    process3 = subprocess.Popen(['/home/piyush/run-bot/run2.sh > /dev/null'], shell=True)
    time.sleep(3)
    process4 = subprocess.call('cd /home/piyush/OpenCV3_Local/test', shell=True)

    # initOdometry()  # Initialize odometry module


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


def readMotorTicks():
    tick = rbt.readTick()
    return (tick[0][1], tick[1][1])


def msg_handler(channel, data):
    global turnAngle
    global distanceToObstacle
    msg = example_t.decode(data)
    print("Received message on channel \"%s\"" % channel)
    turnAngle = msg.angle
    print("Target Angle = %s" % str(turnAngle))
    distanceToObstacle = msg.distance
    print("Distance to obstacle = %s" % str(distanceToObstacle))


def main():
    global calibrateMode
    global turnMode

    lc = lcm.LCM()  # Create object of lcm library
    subscription = lc.subscribe("PROCESSING_RECEIVE", msg_handler)

    print "Getting pyfly2 context ..."
    context = pyfly2.Context()
    print "Done."

    # assert(not (context.num_cameras == 0))
    print "Getting camera ...",
    camera = context.get_camera(0)
    print "Done."

    print "Connecting to camera ...",
    camera.Connect()
    print "Done."

    print "Querying camera information ..."
    for k, v in camera.info.iteritems():
        print k, v
    print "Done."

    print "Starting capture mode ...",
    camera.StartCapture()
    print "Done."

    start_time = time.time()
    counter = 0

    turnAngle_Ref = mc.getYaw()
    print "Reference angle:", turnAngle_Ref

    while(True):
        end_time = time.time()
        if(end_time - start_time > PROCESSING_INTERVAL):
            stopBot()  # Stop the bot during processing
            frame = camera.GrabNumPyImage('bgr')
            # TODO: Undistort frame
            cv2.imwrite("share/" + str(counter) + ".jpg", frame)
            print "Written to share/" + str(counter) + ".jpg"
            msg = example_t()
            msg.currImage = counter
            # TODO add focal length to message / result
            lc.publish("PROCESSING_SEND", msg.encode())
            print "published by gc.py"
            # TODO Delete previous image
            lc.handle()     # Recieve angle
            start_time = time.time()    # Restart count

        counter += 1
        global turnAngle
        ANGLE_THRES = 0.13
        angle_to_rotate = turnAngle 
        print "Angle to rotate:", angle_to_rotate
        if(abs(angle_to_rotate) > ANGLE_THRES):    # Turn bot (sharply) in given direction
            rotate(angle_to_rotate * 180 / math.pi)
        else:
            moveForward()

        # TODO handle bot movement
        time.sleep(SLEEP_TIME)


if __name__ == '__main__':
    # orient()
    # init()
    initOdometry()  # Initialize odometry module
    main()
