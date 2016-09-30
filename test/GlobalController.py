# from detect_aruco_image import *
from detect_aruco_flycam import *
import time
import sys
import os
import cgbot
from cgbot.commands import cmd
import subprocess
from imports import WrapperForPointGray

BOT_SPEED = 5000
CUTOFF_DISTANCE = 1.0
SLEEP_TIME = 0.01


def init():
    process1 = subprocess.call('cd /home/piyush/run-bot', shell=True)
    # process1 = Popen(['cd', '/home/piyush/run-bot'])
    process2 = subprocess.Popen(['/home/piyush/run-bot/run1.sh > /dev/null'], shell=True)
    process3 = subprocess.Popen(['/home/piyush/run-bot/run2.sh > /dev/null'], shell=True)
    time.sleep(3)
    process4 = subprocess.call('cd /home/piyush/OpenCV3_Local/test', shell=True)


def main():

    cmd.forward(speed=BOT_SPEED)
    time.sleep(SLEEP_TIME)

    while(True):

        # get an image
        # markers = findImageArucoParams('ar2.png')
        markers = getMarkersFromCurrentFrame()

        # Process markers
        print "Markers detected", len(markers)
        if len(markers) > 0:
            for marker in markers:
                # marker.draw(img, np.array([255, 0, 0]), 100, True)
                # for i, point in enumerate(marker):
                #     print i, point

                marker.calculateExtrinsics(marker_size, camparam)

                print "Id:", marker.id
                print "Rvec:\n", marker.Rvec
                print "Tvec:\n", marker.Tvec

                # make an action according to detected marker.
                if(marker.Tvec[2][0] < CUTOFF_DISTANCE):
                    cmd.stop()

            # sleep
            time.sleep(SLEEP_TIME)


if __name__ == '__main__':
    init()
    main()
