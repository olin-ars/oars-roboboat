import numpy as np
"""
Helper functions for creating 'votes' for the arbiter
Includes functions to build gaussian distributions
"""

def gaussian(x, center, width, peak):
	""" Gaussian Distribution Function """
	num=-(x-center)**2.
	den=2.*width**2
	return peak*np.exp(num/den)

def makeGauss(center, width=20, peak=0.8, length=101):
	""" 
		returns a gaussian distribution of 'votes' in a vote sized list
		centered at "center" indices from the middle of the list 
		with input width and peak hieght
	"""
	res = []
	for i in range(-length/2, length/2):
		res.append(gaussian(i, center, width, peak))
	return res

def max_speed(speed):
	return [speed]*101