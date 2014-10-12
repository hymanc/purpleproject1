# Python 2.7 Laserbot Vision System
# EECS 498 Purple Team, 2014
# Written by Cody Hyman (hymanc@umich.edu)
# Written against OpenCV 3.0.0-alpha

import sys
import os

import cv2
import numpy as np

from uvcinterface import UVCInterface as uvc

# Calibration state 'Enumeration'
class CalState(object):
    UNCAL = 1
    CAL_PROG = 2
    CALIBRATED = 3

class VisionSystem(object):
	# Window names
	CAM_FEED_NAME = 'Camera Feed'
	CAL_NAME = 'Calibrated Image'
	PROC1_NAME = 'Processing 1'
	PROC2_NAME = 'Processing 2'

	G_CENTER = 52
	R_CENTER = 0
	SMIN = 50
	VMIN = 80

	def __init__(self, camera):
		self.camera = camera
		self.calstate = CalState.UNCAL
		self.calpts = []
		self.XSIZE = 600
		self.YSIZE = 600
		self.worldpts = np.float32([[0,0],[self.XSIZE,0],[self.XSIZE,self.YSIZE],[0,self.YSIZE]])
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
	    # Camera window
		camWindow = cv2.namedWindow(self.CAM_FEED_NAME)
		cv2.createTrackbar('Gain', self.CAM_FEED_NAME, 128, 255, self.gainChanged)
		cv2.createTrackbar('Exposure', self.CAM_FEED_NAME, 1600, 2000, self.exposureChanged)
		cv2.createTrackbar('Saturation', self.CAM_FEED_NAME, 128, 255, self.saturationChanged)
		# Set mouse callbacks for calibration
		cv2.setMouseCallback(self.CAM_FEED_NAME, self.mouseClickHandler)
		
		# Rectified/Calibrated Image window
		calWindow = cv2.namedWindow(self.CAL_NAME)
		cv2.setMouseCallback(self.CAL_NAME, self.colorClickHandler)

		# Image processing window 1
		procWindow1 = cv2.namedWindow(self.PROC1_NAME)
		cv2.createTrackbar('Threshold', self.PROC1_NAME, 0, 255, self.trackbarChangeHandler)
		
		# Image processing Window 2
		procWindow2 = cv2.namedWindow(self.PROC2_NAME)
		cv2.createTrackbar('Green', self.PROC2_NAME, 58, 255, self.trackbarChangeHandler) 
		cv2.createTrackbar('Red', self.PROC2_NAME, 0, 255, self.trackbarChangeHandler)
		cv2.createTrackbar('GCutoff', self.PROC2_NAME, 80, 255, self.trackbarChangeHandler)
		cv2.createTrackbar('RCutoff', self.PROC2_NAME, 100, 255, self.trackbarChangeHandler)

		# Main processing loop
		while(True):
		    frameRet, self.camImg = self.vidcap.read()
		    self.drawCalMarkers()
		    cv2.imshow(self.CAM_FEED_NAME, self.camImg)
		    if(self.calstate == CalState.CALIBRATED):
				self.remapImage()
				#self.segmentImage()
				gr = cv2.getTrackbarPos('Green', self.PROC2_NAME)
				rd = cv2.getTrackbarPos('Red', self.PROC2_NAME)
				gvmin = cv2.getTrackbarPos('GCutoff', self.PROC2_NAME)
				rvmin = cv2.getTrackbarPos('RCutoff', self.PROC2_NAME)
				gCentroid, self.gTagImg = self.findMarker(self.warpImg, gr, 10, self.SMIN, gvmin)
				rCentroid, self.rTagImg = self.findMarker(self.warpImg, rd, 10, self.SMIN, rvmin)
				print 'Green:', str(gCentroid), ' Red:', str(rCentroid)
				#self.printCentroids(gCentroid, rCentroid)
				self.rgImg = self.comboImage(self.gTagImg, self.rTagImg)
				cv2.imshow(self.CAL_NAME, self.warpImg)
				cv2.imshow(self.PROC1_NAME, self.rTagImg)
				cv2.imshow(self.PROC2_NAME, self.rgImg)
		    if cv2.waitKey(20) & 0xFF == ord('q'):
			break
    

	# Use current perspective transform to remap image
	def remapImage(self):
		if(self.calstate == CalState.CALIBRATED):
			self.warpImg = cv2.warpPerspective(self.camImg, self.warp,(self.XSIZE,self.YSIZE))
			self.warpImg = cv2.medianBlur(self.warpImg, 5)
		else:
		    print 'Transform not calibrated'

	#def printCentroids(self, gCenter, rCenter):
		#pass
		#print '"Green:({:d},{:d})'.format(*gCenter)

	def drawSquareMarker(self, img, x , y, width, color):
		w2 = width/2
		cv2.rectangle(img, (x-w2,y-w2),(x+w2,y+w2),color,1)
	       
	def drawCalMarkers(self):
		for pt in self.calpts:
		    self.drawSquareMarker(self.camImg, pt[0], pt[1], 5, (255,0,255))
	
	def comboImage(self, gImg, rImg):
		zeroArr = np.zeros(gImg.shape, dtype=np.uint8)
		return cv2.merge((zeroArr,gImg, rImg))

	# Finds a marker's central moment
	def findMarker(self, image, hueCenter, hueWidth, satMin, valMin):
		hsvImg = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		markerImg = cv2.inRange(hsvImg, np.array([hueCenter-hueWidth/2, satMin, valMin]), np.array([hueCenter+hueWidth/2, 255, 255]))
		cleanElement = cv2.getStructuringElement(cv2.MORPH_CROSS, (5,5))
		markerImg = cv2.erode(markerImg, cleanElement) # Clean up marker image w/ erode-dilate-median
		markerImg = cv2.dilate(markerImg, cleanElement)
		markerImg = cv2.medianBlur(markerImg, 5)
		mMoments = cv2.moments(markerImg) # Compute moments
		m00 = mMoments['m00']
		if(m00 > 0.1):
			return (mMoments['m10']/m00, mMoments['m01']/m00), markerImg
		return None, markerImg

	def segmentImage(self):
		grayImage = cv2.cvtColor(self.warpImg, cv2.COLOR_BGR2GRAY) # Convert to grayscale
		#blurImage = cv2.GaussianBlur(grayImage, (5,5), 0) # Prelim filtering
		blurImage = cv2.medianBlur(grayImage, 5)
		otsuValue = cv2.getTrackbarPos('Threshold', self.PROC1_NAME)
		center = cv2.getTrackbarPos('Green', self.PROC2_NAME)
		width = cv2.getTrackbarPos('Red', self.PROC2_NAME)
		cutoff = cv2.getTrackbarPos('Cutoff', self.PROC2_NAME)
		#valmin = cv2.getTrackbarPos('ValMin', self.PROC2_NAME)
		r1, self.segImg1 = cv2.threshold(blurImage, otsuValue, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU) #Otsu Binarization
		#self.segImg2 = cv2.adaptiveThreshold(blurImage, adaptiveValue, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
		hsvImg = cv2.cvtColor(self.warpImg, cv2.COLOR_BGR2HSV)
		self.segImg2 = cv2.inRange(hsvImg, np.array([center-width/2, cutoff, cutoff]), np.array([center+width/2,255,255]))

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

  