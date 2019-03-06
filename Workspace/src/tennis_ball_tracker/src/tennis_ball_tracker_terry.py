# import necessary packages
import cv2
import numpy as np
import imutils
from imutils.video import VideoStream
import time
from collections import deque

# define the lower and upper boundaries for green
greenLow = np.array([25, 39, 130])
greenHigh = np.array([88, 255, 255])

# initialize list of tracked points with buffer of 50
pts = deque(maxlen=50)

# grab video stream to webcam
vs = cv2.VideoCapture(0)

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4',fourcc, 10.0, (1920,1080))

# allow the camera or video file to warm up
time.sleep(2.0)

# keep looping
while True:
    # grab the current frame
    frame = vs.read()[1]

    # crop to only use left view
    height, width = frame.shape[:2]
    #frame = frame[:, 0:int(width / 2.0)]
    frame = imutils.resize(frame, width=600)

    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    if frame is None:
        break

    # apply bilateral filter to preserve edges
    #blurred = cv2.bilateralFilter(frame, 9, 75, 75)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)

    # convert to hsv colour space
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # threshold image to find green tennis ball
    mask = cv2.inRange(hsv, greenLow, greenHigh)

    # perform a series of dilations and erosions to remove noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, None, iterations=1)

    # apply closing to fill in gaps such as seam
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, None, iterations=1)

    # find contours in thresholded image
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    # iterate through contours to find circles
    valid_contours = list()
    valid_coords = list()

    for c in cnts:
        # get approximate number of vertices
        approx = cv2.approxPolyDP(c, 0.01 * cv2.arcLength(c, True), True)

        # discard non-circle contours
        if len(approx) > 8 and len(approx) < 18 and cv2.contourArea(c) > 20:
            valid_contours.append(c)

    # find largest contour
    if len(valid_contours) > 0:
        max_cnt = max(valid_contours, key=cv2.contourArea)
        M = cv2.moments(max_cnt)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # update the points queue
        pts.appendleft((cX, cY))

        # draw crosshairs on image
        height, width = frame.shape[:2]
        cv2.line(frame, (int(width / 2.0), 0), (int(width / 2.0), height), (255, 0, 255), 3)
        cv2.line(frame, (0, int(height / 2.0)), (width, int(height / 2.0)), (255, 0, 255), 3)

        # determine minimum enclosing circle for countour
        ((x, y), radius) = cv2.minEnclosingCircle(max_cnt)
        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 0, 255), 3)
        cv2.circle(frame, (int(x), int(y)), 7, (255, 0, 0), 3)

        # loop over the set of tracked points
        for i in range(1, len(pts)):
            # if either of hte tracked points are None, ignore them
            if pts[i - 1] is None or pts[i] is None:
                continue

            # otherwise, compute the thickness of the line and
            # draw connecting lines
            thickness = int(np.sqrt(50 / float(i + 1)) * 2.5)
            cv2.line(frame, pts[i-1], pts[i], (0, 0, 255), thickness)

    # show the frame to our screen
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    # resize frame and write to file
    frame_w = cv2.resize(frame, (1920, 1080))
    frame_w = cv2.flip(frame_w, 1)
    out.write(frame_w)

    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

vs.release()
out.release()

# close all windows
cv2.destroyAllWindows()