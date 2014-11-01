import os
import sys
sys.path.insert(0, os.getcwd())

from joy import JoyApp
from joy.decl import *
from fixed_vision.visionPlan import VisionPlan
from fixed_vision.sensorPlan import SensorPlan
from doritoControl.doritoMotion import DoritoMotion
import numpy as np

# Dorito state enumeration class
class DoritoState(object):
    UNKNOWN = 'UNKNOWN'
    SETUP = 'SETUP'
    RUNNING = 'RUNNING'
    FINISHED = 'FINISHED'
    
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
	
	# Attempt to start vision system
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
	
	# Set timer for events
	self.timeForControl = self.onceEvery(0.1)
	
    # Top Level Event Handler (Key commands?)
    def onEvent( self , evt ):
	#print 'Event Occurred', str(evt)
	if self.timeForControl(): 
	    # Check
	    print 'Handling Control Update'
	    wp = (self.sensor.lastWaypoints)[1] # Update Waypoints list
	    currState = self.vplan.getState()		# Get latest state feedback from vision system
	    print 'Current state:', str(currState)
	    if(len(wp) > 0): # If waypoint exists
		currWp = wp[0]
		print 'Next Waypoint:', str(currWp) # 
		# Get next feedback
		# Compute next Control
		# Send next command
	    else:
		print 'No more waypoints available'
	    
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
	    self._parseControls()
	
	
	return JoyApp.onEvent(self,evt)
	
	
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
	self.drive.setSpeed(np.asarray(f), t)
	   
	
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
    