import cv2
import numpy as np
import time
import sys
from timeit import default_timer as timer

def CheckGround():

    EdgeArray = []
    PointArray = []

    time.sleep(0.1)  # let image settle
    # ret, img = capture.read()  # get a bunch of frames to make sure current frame is the most recent
    # ret, img = capture.read()
    # ret, img = capture.read()
    # ret, img = capture.read()
    # ret, img = capture.read()  # 5 seems to be enough
    img = cv2.imread('floor_4.jpg')
    # print img.shape

    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # convert img to grayscale and store result in imgGray

    # imgGray = cv2.GaussianBlur(imgGray, (11, 11), 0)
    # cv2.imwrite('floor_gray.jpg', imgGray)
    # imgThreshold = cv2.adaptiveThreshold(imgGray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    # cv2.imwrite('floor_thres.jpg', imgThreshold)
    # lines = cv2.HoughLines(imgGray, 1, np.pi/180, 3000, 0, 0)
    # print lines.shape
    # cv2.imwrite('floor_lines.jpg', lines)
    # imgGray = cv2.adaptiveBilateralFilter(imgGray, (9, 9), 50)
    # cv2.imwrite('floor_bilat.jpg', imgGray)
    imgGray = cv2.medianBlur(imgGray, 11)
    imgGray = cv2.bilateralFilter(imgGray, 11, 50, 50)  # blur the image slightly to remove noise

    imgEdge = cv2.Canny(imgGray, 50, 100)  # edge detection
    cv2.imwrite('floor_edges.jpg', imgEdge)
    # sys.exit()

    imagewidth = img.shape[1]
    imageheight = img.shape[0]
    # print imagewidth, imageheight

    # Detect empty regions in image
    start_time = timer()
    WindowSize = 200
    StepSize = 40
    # num_iter_x = int((imagewidth - WindowSize + 1) / (StepSize + 1) + 1)
    # num_iter_y = int((imageheight - WindowSize + 1) / (StepSize + 1) + 1)
    N = int(WindowSize / StepSize)
    w = int(imagewidth / StepSize)
    h = int(imageheight / StepSize)
    # print w, h
    # dp = np.full((w, h), -1)
    fCount = np.full((w, h), -1)    # Count floor pixels
    nfCount = np.full((w, h), -1)   # Count non-floor pixels
    FLOOR_THRES = 15
    EMPTY_THRES = 0.02
    for i in range(w - 1):
        for j in range(h - 1):
            # Count number of floor and non-floor pixels in box
            # Floor assumed to be white - all colors near avg
            # isEmptyBox = False
            countNFloor = 0
            countFloor = 0
            start_i = i * StepSize
            end_i = start_i + StepSize
            start_j = j * StepSize
            end_j = start_j + StepSize
            for k in range(start_i, end_i, 1):
                for l in range(start_j, end_j, 1):
                    pixel = img[l, k]
                    avgColor = pixel[0]/3 + pixel[1]/3 + pixel[2]/3
                    if(abs(pixel[0] - avgColor) < FLOOR_THRES and abs(pixel[1] - avgColor) < FLOOR_THRES and abs(pixel[2] - avgColor) < FLOOR_THRES):
                        countFloor = countFloor + 1
                    else:
                        countNFloor = countNFloor + 1
            # if(float(countWhite) / float(countBlack) < EMPTY_THRES):
            #     isEmptyBox = True
            # dp[i, j] = int(isEmptyBox)
            fCount[i, j] = countFloor
            nfCount[i, j] = countNFloor
    end_time = timer()
    print "Count loop time: ", (end_time - start_time)

    # countOnes = 0
    # countZeros = 0
    # for i in range(w):
    #     for j in range(h):
    #         if dp[i, j] == 0:
    #             countZeros = countZeros + 1
    #         if dp[i, j] == 1:
    #             countOnes = countOnes + 1
    # print "Zeros: ", countZeros
    # print "Ones: ", countOnes

    start_time = timer()
    for i in range(w - N):
        for j in range(h - N):
            # Count number of floor and non-floor pixels in window
            # isEmptyWindow = True
            countFloor = 0
            countNFloor = 0
            for k in range(i, i + N, 1):
                for l in range(j, j + N, 1):
                    # if (dp[k, l] == 0):
                    #     isEmptyWindow = False
                    #     break
                    countFloor = countFloor + fCount[k, l]
                    countNFloor = countNFloor + nfCount[k, l]
            # Add center of point array if window has sufficient floor points
            # if(isEmptyWindow):
            if (float(countNFloor)/float(countNFloor + countFloor) < EMPTY_THRES):
                PointArray.append(((i + N / 2) * StepSize, (j + N / 2) * StepSize))
    end_time = timer()
    print "Segmentation loop time: ", (end_time - start_time)

    print PointArray

    # draw points on image
    for point in PointArray:
        cv2.circle(img, point, 10, (0, 255, 0), -1)

    # # Detect white pixels in line
    # for j in range(0, imagewidth, StepSize):  # for the width of image array
    #     for i in range(imageheight - 5, 0, -1):  # step through every pixel in height of array from bottom to top
    #                                             # Ignore first couple of pixels as may trigger due to undistort
    #         if imgEdge.item(i, j) == 255:  # check to see if the pixel is white which indicates an edge has been found
    #             EdgeArray.append((j, i))  # if it is, add x,y coordinates to ObstacleArray
    #             break  # if white pixel is found, skip rest of pixels in column
    #     else:  # no white pixel found
    #         # if nothing found, assume no obstacle. Set pixel position way off the screen to indicate
    #         EdgeArray.append((j, 0))
    #         # no obstacle detected
    #
    # for x in range(len(EdgeArray) - 1):  # draw lines between points in ObstacleArray
    #     cv2.line(img, EdgeArray[x], EdgeArray[x + 1], (0, 255, 0), 1)
    # for x in range(len(EdgeArray)):  # draw lines from bottom of the screen to points in ObstacleArray
    #     cv2.line(img, (x * StepSize, imageheight), EdgeArray[x], (0, 255, 0), 1)

    cv2.imwrite('floor_final.jpg', img)
    print "Written to floor_final.jpg"

if __name__ == '__main__':
    CheckGround()
