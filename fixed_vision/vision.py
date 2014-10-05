# Python 2.7 Laserbot Vision System
# EECS 498 Purple Team, 2014
# Written by Cody Hyman (hymanc@umich.edu)
# Written against OpenCV 3.0.0-alpha

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
	self.XSIZE = 800
	self.YSIZE = 800
	self.worldpts = np.float32([[0,0],[0,self.YSIZE],[self.XSIZE,self.YSIZE],[self.XSIZE,0]])
	print 'Opening Camera ' + str(camera)
	vidcap = cv2.VideoCapture(camera)# Open up specified camera
	camWindow = cv2.namedWindow('Camera Feed')
	procWindow = cv2.namedWindow('Vision Processing')
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
		cv2.imshow('Vision Processing', self.warpImg)
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
    vs = VisionSystem(0)
    vidcap.release()
    cv2.release()
    cv2.destroyAllWindows()
	    
	    
if __name__ == '__main__':
    main()

  