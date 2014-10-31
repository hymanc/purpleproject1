# Vision system static utilities

import cv2
import numpy as np
from math import atan2

# Vision utilities class
class VisionUtil(object):
    
	# Prints marker centroids
	@staticmethod
	def printCentroids(gCtr, rCtr):
		if((gCtr != None) and (rCtr != None)):
			gx = round(gCtr[0],2)
			gy = round(gCtr[1],2)
			rx = round(rCtr[0],2)
			ry = round(rCtr[1],2)
			print 'Green:(', str(gx), ',', str(gy), ') \t Red:(', str(rx), ',', str(ry), ')'

	# Creates a red/green combination image from filter data
	@staticmethod
	def comboImage(bImg=None, gImg=None, rImg=None, background=None):
		zeroArr = np.zeros(gImg.shape, dtype=np.uint8)
		if(bImg == None):
		    bImg = zeroArr
		if(gImg == None):
		    gImg = zeroArr
		if(rImg == None):
		    rImg = zeroArr
		combo = cv2.merge((bImg, gImg, rImg))
		if(background != None):
		    #combo = cv2.bitwise_or(cv2.bitwise_not(cv2.bitwise_and(combo, background)), combo)
		    combo = cv2.add(background, combo)
		return combo

	# Draws square marker on an image
	@staticmethod
	def drawSquareMarker(img, x , y, width, color):
		w2 = width/2
		cv2.rectangle(img, (x-w2,y-w2),(x+w2,y+w2),color,1)

	# Finds
	@staticmethod
	def localizeRobot(gCtr = None, rCtr = None, bCtr = None, printFlag = False):
		if((gCtr != None) and (rCtr != None)):
			ctr = ((gCtr[0] + rCtr[0])/2, (gCtr[1] + rCtr[1])/2) # Compute line midpoint
			theta = atan2(gCtr[1]-rCtr[1], gCtr[0]-rCtr[0]) # Compute angle
			if(printFlag):
			    ctrStr = '(' + str(round(ctr[0],1)) + ',' + str(round(ctr[1],1)) + ')'
			    thetaStr = str(round(theta,3))
			    print 'Center:', ctrStr, '\tTheta:', thetaStr
			return ctr, theta
		else:
			return None, None
			
