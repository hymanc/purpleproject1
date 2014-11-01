# Vision system static utilities

import cv2
import numpy as np
#from math import atan2
from math import *

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
	def localizeRobot(bCtr = None, gCtr = None, rCtr = None, printFlag = False):
		ctr = None
		theta = None
		
		# Determine which tags are visible
		if(bCtr != None and gCtr != None and rCtr != None): # All three tags visible
		   #ctr = (np.mean([bCtr[0], gCtr[0], rCtr[0]]),np.mean([bCtr[1], gCtr[2], rCtr[3]]))
		   aGR = atan2(rCtr[1] - gCtr[1], rCtr[0] - gCtr[0]) # Compute green-red angle
		   aBR = atan2(rCtr[1] - bCtr[1], rCtr[0] - bCtr[0]) # Compute blue-red angle
		   aBG = atan2(gCtr[1] - bCtr[1], gCtr[0] - bCtr[0]) # Compute blue-green angle
		   theta = np.mean([aGR + pi/3, aBR, aBG - pi/3]) # Compute theta estimate from angles
		 
		else: # Determine localization with only two points
		    
		    if(bCtr != None and gCtr != None): # Blue and Green visible, Red unknown
			theta = atan2(bCtr[1] - gCtr[1], bCtr[0] - gCtr[0]) - pi/3 # -pi/3 offset needed
			rDist = sqrt( pow(bCtr[0]-gCtr[0] , 2) + pow(bCtr[1]-gCtr[1],  2) ) # Distance between points
			rAngle = theta # Angle between blue and red points
			rCtr = (gCtr[0] + rDist * cos(rAngle), gCtr[1] + rDist * sin(rAngle)) # Estimated red center
		    
		    elif(bCtr != None and rCtr != None): # Blue and Red Visible, Green Unknown
			theta = atan2(bCtr[1] - rCtr[1], bCtr[0] - rCtr[0]) 
			gDist = sqrt( pow(rCtr[0]-bCtr[0], 2) + pow(rCtr[1]-bCtr[1], 2) ) # Distance between points
			gAngle = theta + pi/3 # Angle between blue and green points
			gCtr = (rCtr[0] + gDist * cos(gAngle), rCtr[1] + gDist * sin(gAngle)) # Estimated green center
		    
		    elif(gCtr != None and rCtr != None): # Green and Red Visible, Blue unknown
			theta = atan2(rCtr[1] - gCtr[1], rCtr[0] - gCtr[0]) + pi/3  # pi/3 offset needed
			bDist = sqrt( pow(rCtr[0]-gCtr[0], 2) + pow(rCtr[1]-gCtr[1], 2) ) # Distance between points
			bAngle = theta - 2*pi/3 # Angle between Green and Blue points
			bCtr = (gCtr[0] + bDist*cos(theta), gCtr[1]  + bDist*sin(theta)) # Estimated blue center
			
		# Compute centroid off real or estimated points
		if(bCtr != None and gCtr != None and rCtr != None):
		    ctr = (np.mean([bCtr[0], gCtr[0], rCtr[0]]),np.mean([bCtr[1], gCtr[1], rCtr[1]]))
		else:
		    print 'Insufficient number of points'
		    ctr = None
		    
		if(printFlag): #Format and print results if requested
		    ctrStr = '(' + str(round(ctr[0],1)) + ',' + str(round(ctr[1],1)) + ')'
		    thetaStr = str(round(theta, 3))
		    print 'Center:', ctrStr, '\tTheta:', thetaStr
		return ctr, theta
			
