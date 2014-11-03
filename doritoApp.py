import os
import sys
sys.path.insert(0, os.getcwd())

from joy import JoyApp
from joy.decl import *
from fixed_vision.visionPlan import VisionPlan
from fixed_vision.visionUtil import VisionUtil as vu
from fixed_vision.sensorPlan import SensorPlan
from doritoControl.doritoMotion import DoritoMotion
import numpy as np
from math import *

# Dorito state enumeration class
class DoritoState(object):
    UNKNOWN = 'UNKNOWN'
    SETUP = 'SETUP'
    RUNNING = 'RUNNING'
    FINISHED = 'FINISHED'
    STOPPED = 'STOPPED'
    MANUAL = 'MANUAL'
    
# Top Level Dorito Robot Class
class DoritoApp( JoyApp ):
    
    # Class constructor
    def __init__(self, camera):
	self.ncam = camera
	self.controls = {'up':False, 'down':False, 'left':False, 'right':False, 'cw':False, 'ccw':False}
	JoyApp.__init__( self, confPath="$/cfg/JoyApp.yml") 

    # Initialization
    def onStart( self ):
	print '\n===== Doritos Xtreme Nacho Cheese Robot Command and Control v99.37 =====\n\n'
	
	# Attempt to start vision system plan
	self.vplan = VisionPlan(self, camera=ncam)
	self.vplan.start()
	
	# Attempt to start drive system
	self.servoErrorFlag = False
	try:
	    self.drive = DoritoMotion() #Initialize Motion DoritoDriver
	except IOError:
	    print '\n*******\nWARNING: Could not connect to drive servos!!!!!\n*******'
	    self.servoErrorFlag = True
	    
	# Attempt to start sensor system
	print 'Starting Sensor Plan\n\n'
	self.sensor = SensorPlan(self)
	self.sensor.start()
	
	self.opState = DoritoState.SETUP
	self.currState = {'x':None,'y':None,'theta':None, 'tagX':None, 'tagY':None}
	# Set timer for events
	self.timeForControl = self.onceEvery(0.1)
	
    # Top Level Event Handler (Key commands?)
    def onEvent( self , evt ):
	#print 'Event Occurred', str(evt)
	if self.timeForControl(): 
	    # Print
	    self.currState = self.vplan.getState()		# Get latest state feedback from vision system
	    self.printState()
	    waypointsRaw = np.float32(self.sensor.lastWaypoints[1])
	    waypoints = self.swapWaypointXY(waypointsRaw)
	    print 'Waypoints:',str(waypoints)
	    self.vplan.setWaypoints(waypoints)# Pass waypoints to vision system
	    self.tagEst = self.robotTagLocation()
	    print 'Tag Estimate', str(self.tagEst)
	    #self.vplan.setTagLocation(self.tagEst)
	    #print 'Current state:', str(currState)
	    if(self.opState == DoritoState.RUNNING): # Only update commands while running
		print 'Handling Control Update'
		print 'Found', len(waypoints), ' waypoints'
		if(len(waypoints) > 0): # If waypoint exists
		    currWp = waypoints[0]
		    print 'Next Waypoint:', str(currWp) # 
		    tError, rError = self.controlHandler(currWp)		# Run control handler
		    print 'X-Y Error', tError
		    #self.vplan.setControlVectorRender(self.tagEst, self.tagEst + tError)
		    print 'Rotation Error', rError
		else:
		    print 'No more waypoints available, holding'
		    if(self.servoErrorFlag == False):
			print self.servoErrorFlag
			#self.drive.setSpeed(np.asfarray([0.,0.]), 0) # Send command requests to the motion drive# Stop
			holdWP = vu.toWaypointCoordinates( np.array( [self.currState['tagX'],self.currState['tagY']] )) # Current tag location in waypoint space
			self.controlHandler(holdWP)# Run controller to hold orientation
			
	# Manual control handling
	if evt.type == KEYUP:
	    if evt.key == K_UP:
		self.controls['up'] = False
	    elif evt.key == K_DOWN:
		self.controls['down'] = False
	    elif evt.key == K_LEFT:
		self.controls['left'] = False
	    elif evt.key == K_RIGHT:
		self.controls['right'] = False
	    elif evt.key == K_PAGEUP:
		self.controls['ccw'] = False
	    elif evt.key == K_PAGEDOWN:
		self.controls['cw'] = False
	    self._parseControls()
	if evt.type == KEYDOWN:
	    if evt.key == K_UP:
		self.controls['up'] = True
	    elif evt.key == K_DOWN:
		self.controls['down'] = True
	    elif evt.key == K_LEFT:
		self.controls['left'] = True
	    elif evt.key == K_RIGHT:
		self.controls['right'] = True
	    elif evt.key == K_PAGEUP:
		self.controls['ccw'] = True
	    elif evt.key == K_PAGEDOWN:
		self.controls['cw'] = True
	    elif evt.key == K_r: # Run mode
		print '=== Set mode to Run ===\n\n'
		self.opState = DoritoState.RUNNING
	    elif evt.key == K_s: # E-stop
		self.opState = DoritoState.STOPPED
		if(self.servoErrorFlag == False):
		    self.drive.stopAll()
	    elif evt.key == K_m: # Manual drive
		self.opState = DoritoState.MANUAL
	    self._parseControls()
	
	return JoyApp.onEvent(self,evt)
	
    # Manual control parser
    def _parseControls(self):
	f = np.array((0.,0.))
	t = 0.
	print 'Controls: ', self.controls
	if(self.controls['up']):
	    f = f + np.array((0., 3.))
	if(self.controls['down']):
	    f = f + np.array((0., -3.))
	if(self.controls['left']):
	    f = f + np.array((-3., 0.))
	if(self.controls['right']):
	    f = f + np.array((3., 0.))
	if(self.controls['ccw']):
	    t = t + 3.
	if(self.controls['cw']):
	    t = t - 3.
	print 'Controls parsed',np.asfarray(f),t
	if(self.servoErrorFlag == False and self.opState == DoritoState.MANUAL):
	    self.drive.setSpeed(np.asarray(f), t)
	   
    # Formatting function for printing the current robot state
    def printState(self):
	x = self.currState['x']
	if(x != None):
	    x = round(x, 3)
	y = self.currState['y']
	if(y != None):
	    y = round(y, 3)
	theta = self.currState['theta']
	if(theta != None):
	    theta = round(theta*180.0/pi, 3)
	tX = self.currState['tagX']
	if(tX != None):
	    tX = round(tX, 3)
	tY = self.currState['tagY']
	if(tY != None):
	    tY = round(tY, 3)
	print 'Current State:\tx:',str(x),'\ty:',str(y),'\t\xCE\xB8:',str(theta)
	print 'TagX:',str(tX),'TagY:',str(tY)
	
    # Controller Handler for 3-axis P-controller
    def controlHandler(self,nextWaypoint):
	# Convert waypoint into image coordinates
	nextWaypoint = vu.toImageCoordinates(nextWaypoint)
	print 'Target waypoint at', str(nextWaypoint), 'in img coordinates'
	# Command scaling factors
	drivescale = 1.0

	# Current state values
	#curX = self.currState['x']
	#curY = self.currState['y']
	curX = self.currState['tagX']
	curY = self.currState['tagY']
	curTheta = self.currState['theta']
	curLoc = np.array((curX,curY))
	# Compute errors
	xyError = nextWaypoint - curLoc
	xyNorm = np.linalg.norm(xyError)
	if(xyNorm > 5):
	    rotscale = 0.8 * np.linalg.norm(xyError)
	else:
	    rotscale = 0.8
	thetaError = -curTheta
	f = drivescale * (xyError) # Net translational "force" command
	t = rotscale*(thetaError) # Net "Torque" command
	if(self.servoErrorFlag == False):
	    self.drive.setSpeed(np.asfarray(f), t) # Send command requests to the motion driver
	return xyError, thetaError # Return errors
	
    # Random walk to hit waypoint if it does not register
    def randomWalk(self, nextWaypoint):
	pass # TODO: Implement
	
    # Estimates tag offselt location give the current robot state
    def robotTagLocation(self):
	tag = (0,0) # Estimate tag 
	x = self.currState['x']
	y = self.currState['y']
	theta = self.currState['theta']
	tagRobot = np.array([[1.],[1.]])
	ct = cos(theta)
	st = sin(theta)
	r = np.array([ [ct, -st],[st, ct] ])
	tagNp = np.dot(r, tagRobot)
	# Convert tagNp to tuple
	tag = (x + tagNp[0][0], x + tagNp[1][0])
	return tag
    
    # Swaps the waypoint x-y coordinates
    def swapWaypointXY(self, waypoints):
	swapWaypoints = []
	for wp in waypoints:
	    swapWaypoints.append((wp[0],wp[1])) #UNDO
	return swapWaypoints
	
# Top level main() bootstrap
if __name__=="__main__":
    import sys
    if len(sys.argv)>1:
	ncam = -1
	for x in range(len(sys.argv)):
	    if(sys.argv[x] == '-c'):
		ncam = int(sys.argv[x+1])
	if ncam == -1:
	    ncam = 0
	app=DoritoApp(ncam)
    else:
	app=DoritoApp(0)
    app.run()
    