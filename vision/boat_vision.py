import cv2
import numpy as np
import sys


def to_greyscale(image):
	return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

def to_boogie_wonderland(image):
	ret,thresh = cv2.threshold(image,127,255,2)
	return thresh

def find_contours_binary(image):
	grey = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	ret, thresh = cv2.threshold(grey,127,255,cv2.THRESH_BINARY)
	contours,h = cv2.findContours(thresh,1,2)
	for cnt in contours:
			cv2.drawContours(grey,[cnt],0,255,2)
	return grey

def avg_distance(input_image, original):
    ''' Returns 100 - hamming distance between two images. It's a lot like a percentage
    	match, but actually nothing like that.'''
    orb = cv2.ORB()
    kp1, des1 = orb.detectAndCompute(input_image, None)
    kp2, des2 = orb.detectAndCompute(original, None)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)
    matches = bf.match(des1,des2)
    matches = sorted(matches, key = lambda x:x.distance)
    if len(matches) > 20:
        return 100 - sum([x.distance for x in matches[0:20]])/20
    elif len(matches) == 0:
        return 0 #For Hamming Distance, 100 is really bad. 
    else:
        return 100 - sum([x.distance for x in matches])/len(matches)

def main():
	webcam_video = cv2.VideoCapture(0)
	red_plus = cv2.imread('red_plus.png')
	while(True):
		# Capture frame-by-frame
		ret, frame = webcam_video.read()
		contours = find_contours_binary(frame)
		cv2.imshow('frame', to_boogie_wonderland(frame))
		match = avg_distance(frame, red_plus)
		print(match)
		#cv2.imshow('frame', to_boogie_wonderland(frame))
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	webcam_video.release()
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
