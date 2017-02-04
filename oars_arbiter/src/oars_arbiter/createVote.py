import numpy as np
"""
Helper functions for creating 'votes' for the arbiter
Includes functions to build gaussian distributions
"""

def gaussian(x, width, peak):
	""" Gaussian Distribution Function """
	num=-(x)**2.
	den=2.*width**2
	return peak*np.exp(num/den)

def makeGauss(center, width=20, peak=0.8, length=101):
	""" 
		returns a gaussian distribution of 'votes' in a vote sized list
		centered at "center" indices from the middle of the list 
		with input width and peak height
	"""
	res = []
	for i in range(-length/2, length/2):
		x = i-center
		res.append(gaussian(x, width, peak))
	return res


def makeWrappedGauss(center, width=20, peak=0.8, length=101):
	""" 
		returns a gaussian distribution of 'votes' in a vote sized list
		centered at "center" indices from the middle of the list 
		with input width and peak height
	"""
	res = []
	for i in range(-length/2, length/2):
		x_main = i-center
		x_wrap = length-abs(x_main)
		x = min([abs(x_main), abs(x_wrap)])
		res.append(gaussian(x, width, peak))
	return res

def directionVoteGauss(center, width=20, peak=0.8):
	return makeWrappedGauss(center, width, peak)

def speedVoteGauss(center, max_speed=100, min_speed=50, width=20):
	return makeWrappedGauss(center, width, max_speed-min_speed)+min_speed

def yawVoteGauss(center, width=20, peak=0.8):
	return makeGauss(center, width, peak, 51)

def max_speed(speed):
	return [speed]*101