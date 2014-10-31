import os
import sys
sys.path.insert(0, os.getcwd())

from joy import JoyApp
from joy.decl import *
from fixed_vision.visionPlan import VisionPlan
from fixed_vision.sensorPlan import SensorPlan
from doritoControl.doritoMotion import DoritoMotion

# Top Level Dorito Robot Class
class DoritoApp( JoyApp ):
    
    # Class constructor
    def __init__(self, camera):
	self.ncam = camera
	JoyApp.__init__( self, confPath="$/cfg/JoyApp.yml") 

    # Initialization
    def onStart( self ):
	print '\n===== Doritos Xtreme Nacho Cheese Robot Command and Control v99.37 =====\n\n'
	#TODO: Control plan
	self.vplan = VisionPlan(self, camera=ncam)
	self.vplan.start()
	self.servoErrorFlag = False
	try:
	    self.drive = DoritoMotion() #Initialize Motion DoritoDriver
	except IOError:
	    print '\n*******\nWARNING: Could not connect to drive servos!!!!!\n*******'
	    self.servoErrorFlag = True
	    
	print 'Starting Sensor Plan\n\n'
	self.sensor = SensorPlan(self)
	self.sensor.start()
	
    # Top Level Event Handler (Key commands?)
    def onEvent( self , evt ):
	#print 'Event Occurred', str(evt)
	if(evt.type == KEYDOWN):
	    print 'Main Keydown event'
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
    