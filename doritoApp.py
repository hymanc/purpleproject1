import os
import sys
sys.path.insert(0, os.getcwd())

from joy import JoyApp
from fixed_vision.visionPlan import VisionPlan

# Top Level Dorito Robot Class
class DoritoApp( JoyApp ):
    
    def __init__(self, camera):
	self.ncam = camera
	JoyApp.__init__( self, confPath="$/cfg/JoyApp.yml") 

	
    def onStart( self ):
	print '\n===== Doritos Xtreme Nacho Cheese Command and Control v99.37 =====\n\n'
	#TODO: Change to feedback plan
	#TODO: Control plan
	self.vplan = VisionPlan(self, camera=ncam)
	self.vplan.start()
	
	
    def onEvent( self ):
	print 'Event!'
	
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
    