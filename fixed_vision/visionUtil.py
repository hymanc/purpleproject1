import cv2
import numpy as np
from math import atan2

class VisionUtil(object):
	@staticmethod
	def printCentroids(gCtr, rCtr):
		if((gCtr != None) and (rCtr != None)):
			gx = round(gCtr[0],2)
			gy = round(gCtr[1],2)
			rx = round(rCtr[0],2)
			ry = round(rCtr[1],2)
			print 'Green:(', str(gx), ',', str(gy), ') \t Red:(', str(rx), ',', str(ry), ')'

	@staticmethod
	def comboImage(gImg, rImg):
		zeroArr = np.zeros(gImg.shape, dtype=np.uint8)
		combo = cv2.merge((zeroArr,gImg, rImg))
		return combo

	@staticmethod
	def drawSquareMarker(img, x , y, width, color):
		w2 = width/2
		cv2.rectangle(img, (x-w2,y-w2),(x+w2,y+w2),color,1)

	@staticmethod
	def localizeRobot(gCtr, rCtr):
		if((gCtr != None) and (rCtr != None)):
			ctr = ((gCtr[0] + rCtr[0])/2, (gCtr[1] + rCtr[1])/2)
			phi = atan2(gCtr[1]-rCtr[1], gCtr[0]-rCtr[0])# Compute angle
			ctrStr = '(' + str(round(ctr[0],1)) + ',' + str(round(ctr[1],1)) + ')'
			phiStr = str(round(phi,3))
			print 'Center:', ctrStr, '\tPhi:', phiStr
