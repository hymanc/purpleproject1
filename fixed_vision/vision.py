# Python 2.7 Doritobot Vision System
# EECS 498 Purple Team, 2014
# Written by Cody Hyman (hymanc@umich.edu)
# Written against OpenCV 3.0.0-alpha

import sys
import os
import threading

import cv2
import numpy as np

from uvcinterface import UVCInterface as uvc
from visionUtil import VisionUtil as vu
from collections import deque

# Calibration state 'Enumeration'
class CalState(object):
    UNCAL = 1
    CAL_PROG = 2
    CALIBRATED = 3

### Vision System Class ###
class VisionSystem(object):
	# Window names
	CAM_FEED_NAME = 'Camera Feed'
	CAL_NAME = 'Calibrated Image'
	PROC_NAME = 'Vision Processing'

	# Constants
	G_CENTER = 52
	R_CENTER = 0
	SMIN = 50
	VMIN = 80

	#HISTORY_LENGTH = 15
	EMPTY_KERNEL = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	RAW_KERNEL = np.array([1, 2, 3, 6, 8, 10, 10, 15, 18, 20], dtype = np.float32)
	FIR_KERNEL = np.multiply(RAW_KERNEL,1/np.linalg.norm(RAW_KERNEL,1)) # Normalized kernel

	def __init__(self, camera):
		### Instance Value initialization ###
		self.camera = camera
		self.calstate = CalState.UNCAL
		self.calpts = []
		self.XSIZE = 600
		self.YSIZE = 600
		self.x_est = -1
		self.y_est = -1
		self.theta_est = -1
		#self.worldpts = np.float32([[0,0],[self.XSIZE,0],[self.XSIZE,self.YSIZE],[0,self.YSIZE]])
		self.worldpts = np.float32([[0,self.YSIZE/2],[0,self.YSIZE],[self.XSIZE,self.YSIZE],[self.XSIZE,self.YSIZE/2]])
		### Camera initialization ###
		print 'Opening Camera ' + str(camera)
		self.vidcap = cv2.VideoCapture(camera)# Open up specified camera
		# Check if camera is opened and exit if not
		if self.vidcap.isOpened():
		    print 'Camera ' + str(camera) + ' opened successfully'
		else:
		    print 'ERROR: Camera ' + str(camera) + ' not opened'
		    return False

		# Set camera autoexposure
		uvc.set(self.camera, uvc.EXPOSURE_AUTO, 1)
		uvc.set(self.camera, uvc.EXPOSURE_AUTO_PRIORITY, 1)

		### Initialize UI elements ###
		# Camera input window
		camWindow = cv2.namedWindow(self.CAM_FEED_NAME)
		cv2.createTrackbar('Gain', self.CAM_FEED_NAME, 128, 255, self.gainChanged)
		cv2.createTrackbar('Exposure', self.CAM_FEED_NAME, 1600, 2000, self.exposureChanged)
		cv2.createTrackbar('Saturation', self.CAM_FEED_NAME, 128, 255, self.saturationChanged)
		cv2.setMouseCallback(self.CAM_FEED_NAME, self.mouseClickHandler) # Set mouse callbacks for calibration
		
		# Rectified/Calibrated Image window
		calWindow = cv2.namedWindow(self.CAL_NAME)
		cv2.setMouseCallback(self.CAL_NAME, self.colorClickHandler)
		
		# Image processing Window 2
		procWindow2 = cv2.namedWindow(self.PROC_NAME)
		cv2.createTrackbar('Green', self.PROC_NAME, 58, 255, self.trackbarChangeHandler) 
		cv2.createTrackbar('Red', self.PROC_NAME, 0, 255, self.trackbarChangeHandler)
		cv2.createTrackbar('GCutoff', self.PROC_NAME, 80, 255, self.trackbarChangeHandler)
		cv2.createTrackbar('RCutoff', self.PROC_NAME, 100, 255, self.trackbarChangeHandler)
		cv2.createTrackbar('SatCutoff', self.PROC_NAME, 100, 255, self.trackbarChangeHandler)

		self.xHistory = deque(self.EMPTY_KERNEL)
		self.yHistory = deque(self.EMPTY_KERNEL)
		self.thetaHistory = deque(self.EMPTY_KERNEL)

	# Run vision on a frame
	def processFrame(self):
	### Main processing loop ###
	#while(True):
	    frameRet, self.camImg = self.vidcap.read()
	    self.drawCalMarkers()
	    cv2.imshow(self.CAM_FEED_NAME, self.camImg)
	    if(self.calstate == CalState.CALIBRATED):
			self.remapImage() # Apply perspective warp
			gr = cv2.getTrackbarPos('Green', self.PROC_NAME)
			rd = cv2.getTrackbarPos('Red', self.PROC_NAME)
			gvmin = cv2.getTrackbarPos('GCutoff', self.PROC_NAME)
			rvmin = cv2.getTrackbarPos('RCutoff', self.PROC_NAME)
			smin = cv2.getTrackbarPos('SatCutoff', self.PROC_NAME)
			gCentroid, self.gTagImg = self.findMarker(self.warpImg, gr, 10, smin, gvmin)
			rCentroid, self.rTagImg = self.findMarker(self.warpImg, rd, 10, smin, rvmin)
			#vu.printCentroids(gCentroid, rCentroid)
			self.rgImg = vu.comboImage(self.gTagImg, self.rTagImg)
			ctr, theta = vu.localizeRobot(gCentroid, rCentroid)
			if((ctr != None) and (theta != None)):
			    fctr, ftheta = self.filterPoints(ctr, theta)
			    self.x = ctr[0]
			    self.y = ctr[1]
			    self.theta = ftheta
			    vu.drawSquareMarker(self.rgImg, int(fctr[0]), int(fctr[1]), 5, (255,0,0))
			if(gCentroid != None):
				vu.drawSquareMarker(self.rgImg, int(gCentroid[0]), int(gCentroid[1]), 5, (255,0,0))
			if(rCentroid != None):
				vu.drawSquareMarker(self.rgImg, int(rCentroid[0]), int(rCentroid[1]), 5, (255,0,0))
			cv2.imshow(self.CAL_NAME, self.warpImg)
			cv2.imshow(self.PROC_NAME, self.rgImg)
	    #if cv2.waitKey(20) & 0xFF == ord('q'):
	    #    break
	
	# Use current perspective transform to remap image
	def remapImage(self):
		if(self.calstate == CalState.CALIBRATED):
			self.warpImg = cv2.warpPerspective(self.camImg, self.warp,(self.XSIZE,self.YSIZE))
			self.warpImg = cv2.GaussianBlur(self.warpImg, (9,9), 1)
			self.warpImg = cv2.medianBlur(self.warpImg, 5)
		else:
		    print 'Transform not calibrated'

	# Draws calibration markers on the camera image       
	def drawCalMarkers(self):
		for pt in self.calpts:
		    vu.drawSquareMarker(self.camImg, pt[0], pt[1], 5, (255,0,255))

	# Finds a marker's central moment
	def findMarker(self, image, hueCenter, hueWidth, satMin, valMin):
		hsvImg = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		markerImg = cv2.inRange(hsvImg, np.array([hueCenter-hueWidth/2, satMin, valMin]), np.array([hueCenter+hueWidth/2, 255, 255]))
		cleanElement = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
		markerImg = cv2.erode(markerImg, cleanElement) # Clean up marker image w/ erode-dilate-median
		markerImg = cv2.dilate(markerImg, cleanElement)
		markerImg = cv2.medianBlur(markerImg, 3)
		mMoments = cv2.moments(markerImg) # Compute moments
		m00 = mMoments['m00']
		if(m00 > 0.1):
			return (mMoments['m10']/m00, mMoments['m01']/m00), markerImg
		return None, markerImg

	# FIR on centers and angles
	def filterPoints(self, ctr, theta):
		if((ctr != None) and (theta != None)):
			if(len(self.xHistory) == len(self.FIR_KERNEL)):
				self.xHistory.popleft()
			if(len(self.yHistory) == len(self.FIR_KERNEL)):
				self.yHistory.popleft()
			if(len(self.thetaHistory) == len(self.FIR_KERNEL)):
				self.thetaHistory.popleft()
			self.xHistory.append(ctr[0])
			self.yHistory.append(ctr[1])
			self.thetaHistory.append(theta)
			xFilter = np.linalg.norm(np.multiply(self.FIR_KERNEL, np.array(self.xHistory)),1)
			yFilter = np.linalg.norm(np.multiply(self.FIR_KERNEL, np.array(self.yHistory)),1)
			thetaFilter = np.linalg.norm(np.multiply(self.FIR_KERNEL, np.array(self.thetaHistory)),1)
			#print 'Filtered Phi:', phiFilter, ' Raw Theta:', theta
			return (xFilter, yFilter), thetaFilter

	# Interface to get current state estimates
	def getState(self):
	    # Give estimated [x,y,theta]
	    return [self.x_est, self.y_est, self.theta_est, 0] # Temporarily keep phi at 0
	    
	### Event Handlers ###
	# Camera input mouseclick handler
	def mouseClickHandler(self, event, x, y, flags, param):
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
	
	# Color click handler for cal window
	def colorClickHandler(self, event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			print 'Checking marker 1 color at ', str(x), ',', str(y)
			pass # Get color at point
		if event == cv2.EVENT_RBUTTONDOWN:
			print 'Checking marker 2 color at ', str(x), ',', str(y)
			pass # Get color at point

	# Generic do-nothing slider handler (for )
	def trackbarChangeHandler(self, x):
		pass

	# Gain slider handler
	def gainChanged(self, gain):
		uvc.set(self.camera, uvc.GAIN, gain)
	
	# Saturation slider handler
	def saturationChanged(self, sat):
		uvc.set(self.camera, uvc.SATURATION, sat)

	# Exposure slider handler
	def exposureChanged(self, exp):
		uvc.set(self.camera, uvc.EXPOSURE_ABS, exp)
		
	def stop(self):
	    self.vidcap.release()
	    cv2.release()
	    cv2.destroyAllWindows()

# Main function to run vision system as standalone
def main():
	print 'Args:' , str(sys.argv)
	for x in range(len(sys.argv)):
		if(sys.argv[x] == '-c'):
			ncam = int(sys.argv[x+1])
	vs = VisionSystem(ncam)
	self.vidcap.release()
	cv2.release()
	cv2.destroyAllWindows()

	    
if __name__ == '__main__':
    main()

  