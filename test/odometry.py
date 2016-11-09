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

SLEEP_TIME = 0.05
BOT_SPEED = 4000
WHEEL_RADIUS = 0.16 #in metres
ENC_TICKS_360 = 1250
TICK_TO_DIST = 2 * math.pi * WHEEL_RADIUS/ ENC_TICKS_360
DELTA = 0.00003;


def init():
    rbt.connect(motorcontroller())
    mc.initsocket()

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
    initYaw =  mc.getYaw()
    cmd.forward(speed=BOT_SPEED)
    if(direction==1):
        cmd.turn(0.1)
    elif(direction==-1):
        cmd.turn(-0.4)

    while(True):
        yaw = mc.getYaw()
        print yaw
        if(abs(initYaw - yaw) > angle ):
            break
    cmd.stop()
    cmd.stop()

def main():
    init()
    #while(True):
     # print mc.getYaw()-17
    #moveDistance(1,1)
    rotate(1,90)


if __name__ == '__main__':
    main()
