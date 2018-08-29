import cv2
import numpy as np
import imutils

ballDetected = False

# Defining lower and upper bounds of green ball in HSV
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)

# greenLower = (25, 0, 0)
# greenUpper = (60, 255, 255)

camera = cv2.VideoCapture(0)

while(True):
	# Grab the current camera frame
	grabbed, frame = camera.read()

	# Resizing the frame
	frame = imutils.resize(frame, width=600)

	# Blurring and changing colour space
	frame_blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	frame_hsv = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2HSV)

	# Green mask
	mask = cv2.inRange(frame_hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	# Finding contours in the mask
	(_, contours, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	ball_center = None
	tennisBalls = frame.copy()

	if contours is not None and len(contours) > 0:
		ballDetected = True

		# Find largest contour
		largest_contour = max(contours, key=cv2.contourArea)
		((ball_x, ball_y), ball_radius) = cv2.minEnclosingCircle(largest_contour)

		# Finding the centroid
		moments = cv2.moments(largest_contour)
		ball_center = (int(moments["m10"]/moments["m00"]), int(moments["m01"]/moments["m00"]))

		# Check radius constraints
		if ball_radius > 10:
			cv2.circle(tennisBalls, (int(ball_x), int(ball_y)), int(ball_radius), (0, 0, 255), 3)
			cv2.circle(tennisBalls, ball_center, 3, (0, 0, 255), -1)
	else:
		ballDetected = False

	cv2.imshow("Webcam", tennisBalls)
	print ballDetected

	# Terminate script if 'q' key is pressed
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

camera.release()
cv2.destroyAllWindows()