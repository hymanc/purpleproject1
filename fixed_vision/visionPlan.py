from vision import *
from joy import Plan

# Vision system as a ckbot Plan
class VisionPlan( Plan ):
    def __init__( self, app, *arg, **kw):
	print 'Args', str(kw)
	self.cameraIndex = kw['camera']
	Plan.__init__(self, app, *arg, **kw ) # Initialize Plan
	
    # Gets the robot vision points
    def getState():
	state = self.vision.getState()
	return {'x':state[0],'y':state[1],'theta':state[2]}
    
    # Vision plan behavior
    def behavior( self ):
	print 'Launching Vision Plan'
	# Check 
	if(self.cameraIndex):
	    print 'Starting vision system with camera', str(self.cameraIndex)
	    self.vision(VisionSystem(self.cameraIndex))
	else:
	    print 'Starting vision system with default camera'
	    self.vision = VisionSystem(0) # start vision system
	