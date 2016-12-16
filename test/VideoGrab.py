import os
import numpy as np
import pyfly2
import cv2


def main():
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

    counter = 0
    N = 40
    while (True):
        if(counter % N == 0):
            frame = camera.GrabNumPyImage('bgr')
            imagename = './obstacle-detection/floor-video/images/' + str(counter/N).zfill(4)
            cv2.imwrite(imagename + ".jpg", frame)
        counter += 1

    print "Stopping capture mode ...",
    camera.StopCapture()
    print "Done."


if __name__ == '__main__':
    main()
