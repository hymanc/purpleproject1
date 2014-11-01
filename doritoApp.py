import os
import sys
sys.path.insert(0, os.getcwd())

from joy import JoyApp
from joy.decl import *
from fixed_vision.visionPlan import VisionPlan
from fixed_vision.sensorPlan import SensorPlan
from doritoControl.doritoMotion import DoritoMotion

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
	
	# Set timer for events
	self.timeForControl = self.onceEvery(0.1)
	
    # Top Level Event Handler (Key commands?)
    def onEvent( self , evt ):
	#print 'Event Occurred', str(evt)
	if self.timeForControl(): 
	    print 'Handling Control Update'
	elif(evt.type == KEYDOWN):
	    print 'Key Pressed'
	    # Get key and check for start/stop condition
	return JoyApp.onEvent(self,evt)
	
	
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
    