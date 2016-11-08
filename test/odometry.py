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
import socket, traceback
import netifaces as ni

SLEEP_TIME = 0.05
BOT_SPEED = 4000
WHEEL_RADIUS = 0.08912 #0.16 #in metres
ENC_TICKS_360 = 1250
TICK_TO_DIST = 2 * math.pi * WHEEL_RADIUS/ ENC_TICKS_360
DELTA = 0.0000;





host = ni.ifaddresses('wlan0')[2][0]['addr']
#print ip  # should print "192.168.100.37
# Note that the IP Address and Port in this script and the script on the Mobile Phone should match.
#host = "10.192.46.131"   #IP Address of this Machine
port = 3400

s = None

def initsocket():
    global s
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind((host, port))
    return s

def getYaw():
    global s
    try:
        message, address = s.recvfrom(4096)
        orient = float((message.split())[1])
        orient = orient * 180/3.14;

        return orient
    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()



def init():
    rbt.connect(motorcontroller())
    initsocket()

def readMotorTicks():
    temp = rbt.readTick()
    return (temp[0][1],temp[1][1])

def moveDistance(direction,dis): # @param direction: 1 -> forward and -1 -> backward

    initleft= readMotorTicks()[0]
    initright = readMotorTicks()[1]
    cmd.forward(speed=BOT_SPEED)
    distDelta = DELTA*BOT_SPEED

    while(True):
        temp=readMotorTicks()
        left= temp[0]
        right=temp[1]
        if((left-initleft)*TICK_TO_DIST > dis-distDelta):
            break

    cmd.stop()
    cmd.stop()
    cmd.stop()

def rotate(direction,angle): # @param direction: 1 -> left and -1 -> right
    initYaw = getYaw()
    if(direction==1):
        cmd.turn(0.4)
    elif(direction==-1):
        cmd.turn(-0.4)

    while(True):
        yaw = getYaw()
        if(abs(initYaw - yaw) > angle ):
            break
    cmd.stop()
    cmd.stop()

def main():
    init()
    #moveDistance(1,1)
    rotate(1,90)
    #while(True):
     #   print getYaw()


if __name__ == '__main__':
    main()
