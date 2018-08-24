import cv2
import numpy as np

greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)

# greenLower = (25, 0, 0)
# greenUpper = (60, 255, 255)

cap = cv2.VideoCapture(0)

while(True):
	ret, frame = cap.read()

	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	# cv2.imshow("HSV", hsv)

	# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	# blurred = cv2.GaussianBlur(hsv, (11, 11), 0)
	# edged = cv2.Canny(blurred, 30, 150)
	# cv2.imshow("Edges", edged)

	(_, contours, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	print len(contours)

	tennisBalls = frame.copy()
	cv2.drawContours(tennisBalls, contours, -1, (0, 255, 0), 2)

	cv2.imshow("Webcam", tennisBalls)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()