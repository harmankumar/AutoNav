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
#import control_servo as cs
import MobileCommunication as mc
import socket
import traceback
import netifaces as ni

SLEEP_TIME = 0.05
BOT_SPEED = 4000
WHEEL_RADIUS = 0.08912  # 0.16 #in metres
ENC_TICKS_360 = 1250
TICK_TO_DIST = 2 * math.pi * WHEEL_RADIUS / ENC_TICKS_360
angleDelta = BOT_SPEED * .001
DELTA = 0.0000

initLeft=0
initRight=0
WHEEL_BASE = 1

host = ni.ifaddresses('wlan0')[2][0]['addr']
# print ip  # should print "192.168.100.37
# Note that the IP Address and Port in this script and the script on the Mobile Phone should match.
# host = "10.192.46.131"   #IP Address of this Machine
port = 3400

s = None


def initsocket():
    global s
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind((host, port))
    return s


def initOdometry():
    rbt.connect(motorcontroller())
    mc.initsocket()


def getYaw():
    global s
    try:
        #message, address = s.recvfrom(4096)
        stuff = s.recvfrom(4096)
        #orient = float((message.split())[1])
        #orient = orient * 180/3.14;
        return stuff
        # return orient
    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()


def readMotorTicks():
    temp = rbt.readTick()
    return (temp[0][1], temp[1][1])


def moveForward():
    cmd.forward(speed=BOT_SPEED)


def stopBot():
    cmd.stop()
    cmd.stop()
    cmd.stop()


def moveDistance(direction, distance):
    """ @param direction: 1 -> forward and -1 -> backward """

    initleft = readMotorTicks()[0]
    initright = readMotorTicks()[1]
    cmd.forward(speed=BOT_SPEED)
    distDelta = DELTA * BOT_SPEED

    while(True):
        temp = readMotorTicks()
        left = temp[0]
        right = temp[1]
        distanceMoved = ((left - initleft) + (right - initright)) * TICK_TO_DIST / 2
        print distanceMoved
        if(distanceMoved > dis - distDelta):
            break

    cmd.stop()
    cmd.stop()
    cmd.stop()

def getXY():
    global initLeft
    global initRight
    temp = readMotorTicks()
    left = temp[0]  - initLeft
    right = temp[1] - initRight
    distance = (left+right) * TICK_TO_DIST /2.0
    theta = (left - right) * TICK_TO_DIST / WHEEL_BASE
    X = distance * math.sin(theta)
    Y = distance * math.cos(theta)
    return (X,Y)
    
def moveTowardsXY(x,y):
    angle = math.atan(x/y)
    dist = math.sqrt(x*x + y*y)
    rotate(angle)
    moveDistance(1,dist)

def rotate(angle):
    """
    angle -> signed double value in degree
    direction: 1 -> left and -1 -> right
    """

    sign = lambda a: (a > 0) - (a < 0)
    direction = sign(angle)
    angle = abs(angle)

    initYaw = mc.getYaw()
    cmd.forward(speed=BOT_SPEED)
    if(direction == 1):
        cmd.turn(0.5)
    elif(direction == -1):
        cmd.turn(-0.5)

    while(True):
        yaw = mc.getYaw()
        print yaw
        if(abs(initYaw - yaw) > angle - angleDelta):
            break
    cmd.stop()
    cmd.stop()

def main():
    init()
    #initleft = readMotorTicks()[0]
    # while(True):
    	# print mc.getYaw()
     #print retval
     #temp = readMotorTicks()
     #print (temp[0]- initleft) * TICK_TO_DIST
    #moveDistance(1,3)
    # rotate(1,90)
    moveTowardsXY(3,3)


if __name__ == '__main__':
    main()
