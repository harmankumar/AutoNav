from detect_aruco_image import *
import time
import sys
import os
import cgbot
from cgbot.commands import cmd
import subprocess

def init():
    subprocess.call('cd /home/piyush/run-bot', shell=True)
    # process1 = Popen(['cd', '/home/piyush/run-bot'])
    process2 = subprocess.Popen(['/home/piyush/run-bot/run1.sh'], shell=True)
    process3 = subprocess.Popen(['/home/piyush/run-bot/run2.sh'], shell=True)
    time.sleep(3)
    process4 = subprocess.call('cd /home/piyush/OpenCV3_Local/test', shell=True)

def main():

    cmd.forward(speed=2000)
    time.sleep(10)
    print "Restart"

    # while(True):
    # # get an image
    # #Process it
    # #make an action.
    # #sleep for 10 ms
    #     markers = findImageArucoParams('ar2.png')
    #
    #
    #     print "Markers detected", len(markers)
    #     for marker in markers:
    #         # marker.draw(img, np.array([255, 0, 0]), 100, True)
    #         # for i, point in enumerate(marker):
    #         #     print i, point
    #
    #         marker.calculateExtrinsics(marker_size, camparam)
    #
    #         print "Id:", marker.id
    #         print "Rvec:\n", marker.Rvec
    #         print "Tvec:\n", marker.Tvec
    #
    #         if(marker.Tvec[2][0] < 0.5):
    #             cmd.stop()
    #
    #
    #     time.sleep(1)




if __name__ == '__main__':
    init()
    main()
