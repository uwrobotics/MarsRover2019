"""
File: video_gui.py

This file contains a Python 3.7 based GUI that allows for the
calibration of the HSV ranges
"""

# import necessary packages
import tkinter as tk
import cv2
import numpy as np
import time
import PIL.Image
import PIL.ImageTk
import imutils
from threading import Thread
from collections import deque


class GUI:
    # initialization method
    def __init__(self, master):
        # create window
        self.root = master
        self.root.configure(background='#282c34')
        self.root.title("Camera Calibration Utility")

        # define colours
        self.white = "#ffffff"
        self.gray = "#282c34"

        # variable for resizing preview windows
        self.resizeFactor = 1.0

        # setup frames
        self.leftFrame = tk.Frame(self.root, bg=self.gray)
        self.rightFrame = tk.Frame(self.root, bg=self.gray)
        self.rightUpperFrame = tk.Frame(self.rightFrame, bg=self.gray)
        self.rightLowerFrame = tk.Frame(self.rightFrame, bg=self.gray)

        self.sliderFrame1 = tk.Frame(self.leftFrame, bg=self.gray)
        self.sliderFrame2 = tk.Frame(self.leftFrame, bg=self.gray)
        self.sliderFrame3 = tk.Frame(self.leftFrame, bg=self.gray)
        self.sliderFrame4 = tk.Frame(self.leftFrame, bg=self.gray)
        self.sliderFrame5 = tk.Frame(self.leftFrame, bg=self.gray)
        self.sliderFrame6 = tk.Frame(self.leftFrame, bg=self.gray)

        # labels
        self.hsvTitleLabel = tk.Label(self.leftFrame, text="HSV Ranges",
                                      bg=self.gray, fg=self.white,
                                      font=("Calibri Bold", 25))
        self.hsvLowerLabel = tk.Label(self.leftFrame, text="Lower",
                                      bg=self.gray, fg=self.white,
                                      font=("Calibri Light", 22))
        self.hsvUpperLabel = tk.Label(self.leftFrame, text="Upper",
                                      bg=self.gray, fg=self.white,
                                      font=("Calibri Light", 22))

        # labels for sliders
        self.sliderLabelH1 = tk.Label(self.sliderFrame1, text="H",
                                      bg=self.gray, fg=self.white,
                                      font=("Calibri Light", 18))
        self.sliderLabelS1 = tk.Label(self.sliderFrame2, text="S",
                                      bg=self.gray, fg=self.white,
                                      font=("Calibri Light", 18))
        self.sliderLabelV1 = tk.Label(self.sliderFrame3, text="V",
                                      bg=self.gray, fg=self.white,
                                      font=("Calibri Light", 18))
        self.sliderLabelH2 = tk.Label(self.sliderFrame4, text="H",
                                      bg=self.gray, fg=self.white,
                                      font=("Calibri Light", 18))
        self.sliderLabelS2 = tk.Label(self.sliderFrame5, text="S",
                                      bg=self.gray, fg=self.white,
                                      font=("Calibri Light", 18))
        self.sliderLabelV2 = tk.Label(self.sliderFrame6, text="V",
                                      bg=self.gray, fg=self.white,
                                      font=("Calibri Light", 18))

        # sliders
        self.sliderH1 = tk.Scale(self.sliderFrame1, from_=0, to=180,
                                 orient='horizontal', bg=self.gray,
                                 activebackground=self.gray,
                                 fg=self.white, highlightbackground=self.gray,
                                 highlightcolor=self.white,
                                 length=350, font=("Calibri Light", 14),
                                 command=self.update_values)
        self.sliderS1 = tk.Scale(self.sliderFrame2, from_=0, to=255,
                                 orient='horizontal', bg=self.gray,
                                 activebackground=self.gray,
                                 fg=self.white, highlightbackground=self.gray,
                                 highlightcolor=self.white,
                                 length=350, font=("Calibri Light", 14),
                                 command=self.update_values)
        self.sliderV1 = tk.Scale(self.sliderFrame3, from_=0, to=255,
                                 orient='horizontal', bg=self.gray,
                                 activebackground=self.gray,
                                 fg=self.white, highlightbackground=self.gray,
                                 highlightcolor=self.white,
                                 length=350, font=("Calibri Light", 14),
                                 command=self.update_values)
        self.sliderH2 = tk.Scale(self.sliderFrame4, from_=0, to=180,
                                 orient='horizontal', bg=self.gray,
                                 activebackground=self.gray,
                                 fg=self.white, highlightbackground=self.gray,
                                 highlightcolor=self.white,
                                 length=350, font=("Calibri Light", 14),
                                 command=self.update_values)
        self.sliderS2 = tk.Scale(self.sliderFrame5, from_=0, to=255,
                                 orient='horizontal', bg=self.gray,
                                 activebackground=self.gray,
                                 fg=self.white, highlightbackground=self.gray,
                                 highlightcolor=self.white,
                                 length=350, font=("Calibri Light", 14),
                                 command=self.update_values)
        self.sliderV2 = tk.Scale(self.sliderFrame6, from_=0, to=255,
                                 orient='horizontal', bg=self.gray,
                                 activebackground=self.gray,
                                 fg=self.white, highlightbackground=self.gray,
                                 highlightcolor=self.white,
                                 length=350, font=("Calibri Light", 14),
                                 command=self.update_values)

        # switch between stereo and single camera view
        self.viewSwitch = tk.Button(self.leftFrame,
                                    text="Switch to Stereo View",
                                    command=self.switch_views,
                                    bg=self.gray, fg=self.white,
                                    activebackground=self.gray,
                                    activeforeground=self.gray,
                                    highlightbackground=self.gray,
                                    highlightcolor=self.white,
                                    font=("Calibri Light", 16))

        # canvas element -- used to show images
        self.upperImageCanvas = tk.Canvas(self.rightUpperFrame, width=711,
                                          height=400, bg=self.gray)
        self.lowerImageCanvas = tk.Canvas(self.rightLowerFrame, width=711,
                                          height=400)

        # general packing
        self.leftFrame.pack(side=tk.LEFT)
        self.rightFrame.pack(side=tk.RIGHT)
        self.rightUpperFrame.pack(side=tk.TOP)
        self.rightLowerFrame.pack(side=tk.BOTTOM)

        # left frame packing
        self.hsvTitleLabel.pack(pady=10)
        self.hsvUpperLabel.pack(pady=(10, 0))

        # H Upper
        self.sliderFrame1.pack(padx=10)
        self.sliderLabelH1.pack(side=tk.LEFT, pady=(20, 0), padx=5)
        self.sliderH1.pack(side=tk.LEFT)

        # S Upper
        self.sliderFrame2.pack(padx=10)
        self.sliderLabelS1.pack(side=tk.LEFT, pady=(20, 0), padx=5)
        self.sliderS1.pack(side=tk.LEFT)

        # V Upper
        self.sliderFrame3.pack(padx=10)
        self.sliderLabelV1.pack(side=tk.LEFT, pady=(20, 0), padx=5)
        self.sliderV1.pack(side=tk.LEFT)

        self.hsvLowerLabel.pack(pady=(10, 0))

        # H Lower
        self.sliderFrame4.pack(padx=10)
        self.sliderLabelH2.pack(side=tk.LEFT, pady=(20, 0), padx=5)
        self.sliderH2.pack(side=tk.LEFT)

        # S Lower
        self.sliderFrame5.pack(padx=10)
        self.sliderLabelS2.pack(side=tk.LEFT, pady=(20, 0), padx=5)
        self.sliderS2.pack(side=tk.LEFT)

        # V Lower
        self.sliderFrame6.pack(padx=10, pady=(0, 20))
        self.sliderLabelV2.pack(side=tk.LEFT, pady=(20, 0), padx=5)
        self.sliderV2.pack(side=tk.LEFT)

        # pack switch to move between single and stereo view modes
        self.viewSwitch.pack()

        # right upper frame packing
        self.upperImageCanvas.pack(expand=tk.YES, fill=tk.BOTH)
        self.lowerImageCanvas.pack()

        # grab video stream
        self.vs = cv2.VideoCapture(0)

        # allow the camera to warm up
        time.sleep(2.0)

        # initialize list for slider positions
        self.slider_pos = [0, 0, 0, 0, 0, 0]

        # define window closing protocol
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # initialize list of tracked points with buffer of 50
        self.pts = deque(maxlen=50)

        # callback id
        self.after_id = None

        # start image processing task
        self.update_thread = Thread(target=self.update_images, args=())
        self.update_thread.daemon = True
        self.update_thread.start()

    # function to update slider positions
    def update_values(self, event):
        # get slider positions and update slider position list
        self.slider_pos = [self.sliderH1.get(), self.sliderS1.get(),
                           self.sliderV1.get(), self.sliderH2.get(),
                           self.sliderS2.get(),
                           self.sliderV2.get()]

    # function to cancel tasks on closing
    def on_closing(self):
        self.vs.release()
        self.root.destroy()
        import sys
        sys.exit()

    # function to display tracking previews
    def update_images(self):
        while True:
            # convert image to hsv
            frame = self.vs.read()[1]

            if frame is not None:
                hsv = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2HSV)

                # apply HSV mask
                mask = cv2.inRange(hsv, np.array(
                    [self.slider_pos[3], self.slider_pos[4],
                     self.slider_pos[5]]),
                                   np.array(
                                       [self.slider_pos[0], self.slider_pos[1],
                                        self.slider_pos[2]]))
                mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

                # push image to canvas
                mask = imutils.resize(mask, height=400)
                self.root.photo = photo = PIL.ImageTk.PhotoImage(
                    image=PIL.Image.fromarray(mask))
                self.upperImageCanvas.create_image(0, 0, image=photo,
                                                   anchor=tk.NW)

                # run tracking
                frame = imutils.resize(frame, height=400)
                height, width = frame.shape[:2]
                frame = frame[:, 0:int(width / self.resizeFactor)]

                # apply bilateral filter to preserve edges
                blurred = cv2.bilateralFilter(frame, 9, 75, 75)

                # convert to hsv colour space
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                # threshold image to find green tennis ball
                mask = cv2.inRange(hsv, np.array(
                    [self.slider_pos[3], self.slider_pos[4],
                     self.slider_pos[5]]),
                                   np.array(
                                       [self.slider_pos[0], self.slider_pos[1],
                                        self.slider_pos[2]]))

                # perform a series of dilations and erosions to remove noise
                mask = cv2.erode(mask, None, iterations=10)
                mask = cv2.dilate(mask, None, iterations=10)

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
                    approx = cv2.approxPolyDP(c, 0.01 * cv2.arcLength(c, True),
                                              True)

                    # discard non-circle contours
                    if len(approx) > 8 and len(approx) < 18 and \
                            cv2.contourArea(c) > 20:
                        # append to valid list
                        valid_contours.append(c)

                        # computer centre of contour
                        M = cv2.moments(c)
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])

                        # add to valid list
                        valid_coords.append([cX, cY])
                        center = (cX, cY)

                # update the points queue
                self.pts.appendleft(center)

                # draw crosshairs on image
                height, width = frame.shape[:2]
                cv2.line(frame, (int(width / 2.0), 0),
                         (int(width / 2.0), height), (255, 0, 255), 3)
                cv2.line(frame, (0, int(height / 2.0)),
                         (width, int(height / 2.0)), (255, 0, 255), 3)

                # draw contours and centres
                for i, c in enumerate(valid_contours):
                    # determine minimum enclosing circle
                    # and centre for countour
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    cv2.circle(frame, (int(x), int(y)), int(radius),
                               (0, 0, 255), 3)
                    cv2.circle(frame, (int(x), int(y)), 7, (255, 0, 0), 3)

                # loop over the set of tracked points
                for i in range(1, len(self.pts)):
                    # if either of  the tracked points are None, ignore them
                    if self.pts[i - 1] is None or self.pts[i] is None:
                        continue

                    # otherwise, compute the thickness of the line and
                    # draw connecting lines
                    thickness = int(np.sqrt(50 / float(i + 1)) * 2.5)
                    cv2.line(frame, self.pts[i - 1], self.pts[i], (0, 0, 255),
                             thickness)

                # convert image to PIL format
                self.root.preview_image = preview_image = \
                    PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(frame))
                self.lowerImageCanvas.create_image(0, 0, image=preview_image,
                                                   anchor=tk.NW)

            time.sleep(0.01)

    # function to switch between single and stereo preview modes
    def switch_views(self):
        # switch between factors of 1.0 (single) and 2.0 (stereo)
        # and update button text
        if self.resizeFactor == 1.0:
            self.resizeFactor = 2.0
            self.viewSwitch.config(text="Switch to Single View")
        else:
            self.resizeFactor = 1.0
            self.viewSwitch.config(text="Switch to Stereo View")


# launch GUI
root = tk.Tk()
gui = GUI(root)
root.mainloop()
