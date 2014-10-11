# Python 2.7 Laserbot Vision System
# EECS 498 Purple Team, 2014
# Written by Cody Hyman (hymanc@umich.edu)
# Written against OpenCV 3.0.0-alpha

import sys

import cv2
import numpy as np

class CalState(object):
    UNCAL = 1
    CAL_PROG = 2
    CALIBRATED = 3

class VisionSystem(object):
    
	def __init__(self, camera):
		self.camera = camera
		self.calstate = CalState.UNCAL
		self.calpts = []
		self.XSIZE = 540
		self.YSIZE = 900
		self.worldpts = np.float32([[0,0],[self.XSIZE,0],[self.XSIZE,self.YSIZE],[0,self.YSIZE]])
		print 'Opening Camera ' + str(camera)
		vidcap = cv2.VideoCapture(camera)# Open up specified camera
		camWindow = cv2.namedWindow('Camera Feed')
		calWindow = cv2.namedWindow('Calibrated Image')
		procWindow = cv2.namedWindow('Segmented Image')
		cv2.setMouseCallback('Camera Feed', self.mouseClickHandler)
		if vidcap.isOpened():
		    print 'Camera ' + str(camera) + ' opened successfully'
		else:
		    print 'ERROR: Camera ' + str(camera) + ' not opened'
		    return False
		# Set mouse callbacks for calibration
		
		while(True):
		    frameRet, self.camImg = vidcap.read()
		    self.drawCalMarkers()
		    cv2.imshow('Camera Feed', self.camImg)
		    if(self.calstate == CalState.CALIBRATED):
			self.remapImage()
			self.segmentImage()
			cv2.imshow('Calibrated Image', self.warpImg)
			cv2.imshow('Segmented Image', self.segImg)
		    if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	    
	# Use current perspective transform to remap image
	def remapImage(self):
		if(self.calstate == CalState.CALIBRATED):
		    self.warpImg = cv2.warpPerspective(self.camImg, self.warp,(self.XSIZE,self.YSIZE))
		else:
		    print 'Transform not calibrated'
	    
	def drawSquareMarker(self, img, x , y, width, color):
		w2 = width/2
		cv2.rectangle(img, (x-w2,y-w2),(x+w2,y+w2),color,1)
	       
	def drawCalMarkers(self):
		for pt in self.calpts:
		    self.drawSquareMarker(self.camImg, pt[0], pt[1], 5, (255,0,255))
		
	def segmentImage(self):
		grayImage = cv2.cvtColor(self.warpImg, cv2.COLOR_RGB2GRAY) # Convert to grayscale
		#blurImage = cv2.GaussianBlur(grayImage, (5,5), 0) # Prelim filtering
		blurImage = cv2.medianBlur(grayImage, 5)
		#r1, self.segImg = cv2.threshold(blurImage, 127, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU) #Otsu Binarization
		self.segImg = cv2.adaptiveThreshold(blurImage, 50, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
	def findMarkers(self):
		a = 0# TODO: Find markers

	def mouseClickHandler(self, event,x,y,flags,param):
		if event == cv2.EVENT_RBUTTONDOWN:
		    print 'Recalibration requested'
		    self.calstate = CalState.CAL_PROG
		    self.calpts = [] # Reset calibration points
		if event == cv2.EVENT_LBUTTONDOWN:
		    print 'Mouse left click event at ' + str(x) + ',' + str(y)
		    if(self.calstate == CalState.UNCAL):
			self.calstate = CalState.CAL_PROG
			print 'Adding calibration point at (' + str(x) + ',' + str(y) + ')'
			self.calpts.append([x,y])
		    elif(self.calstate == CalState.CAL_PROG):
			if(len(self.calpts) < 4):
			    print 'Adding calibration point at (' + str(x) + ',' + str(y) + ')'
			    self.calpts.append([x,y])
			    # Finish
			    if(len(self.calpts) == 4):
				print 'Calibrated'
				self.warp = cv2.getPerspectiveTransform(np.float32(self.calpts), self.worldpts)
				print str(self.calpts)
				self.calstate = CalState.CALIBRATED
		    elif(self.calstate == CalState.CALIBRATED):
			print 'Already calibrated'	 

def main():
	print 'Args:' , str(sys.argv)
	for x in range(len(sys.argv)):
		if(sys.argv[x] == '-c'):
			ncam = int(sys.argv[x+1])
	vs = VisionSystem(ncam)
	vidcap.release()
	cv2.release()
	cv2.destroyAllWindows()

	    
if __name__ == '__main__':
    main()

  