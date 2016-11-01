import cv2
import numpy as np
import sys


def to_greyscale(image):
	return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

def to_boogie_wonderland(image):
	ret,thresh = cv2.threshold(image,127,255,1)
	return thresh

def find_contours_binary(image):
	grey = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	ret, thresh = cv2.threshold(grey,127,255,cv2.THRESH_BINARY)
	contours,h = cv2.findContours(thresh,1,2)
	return contours

def main():
	webcam_video = cv2.VideoCapture(0)
	#This closes everything down, so that I don't accidentally break Ubuntu again
	while(True):
		# Capture frame-by-frame
		ret, frame = webcam_video.read()
		grey = to_greyscale(frame)
		contours = find_contours_binary(frame)
		for cnt in contours:
			cv2.drawContours(grey,[cnt],0,255,2)
		cv2.imshow('frame', grey)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	webcam_video.release()
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
