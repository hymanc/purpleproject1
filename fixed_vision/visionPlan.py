from vision import *
from joy import Plan

# Vision system as a ckbot Plan
class VisionPlan( Plan ):
    def __init__( self, app, *arg, **kw ):
	# Get arguments
	Plan.__init__(self, app, *arg, **kw ) # Initialize Plan
	
    # Gets the robot vision points
    def getState():
	state = self.vision.getState()
	return {'x':state[0],'y':state[1],'theta':state[2]}
    # 
    def behavior( self ):
	print 'Vision Plan'
	self.vision = VisionSystem(0) # start vision system