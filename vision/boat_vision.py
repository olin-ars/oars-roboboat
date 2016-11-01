import cv2
import numpy as np
import sys

webcam_video = cv2.VideoCapture(0)

while(True):
	# Capture frame-by-frame
	ret, frame = webcam_video.read()
	cv2.imshow('frame', frame)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

#This closes everything down, so that I don't accidentally break Ubuntu again
webcam_video.release()
cv2.destroyAllWindows()