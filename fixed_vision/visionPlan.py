from vision import *
from joy import Plan
import cv2

# Vision system as a ckbot Plan
class VisionPlan( Plan ):
    def __init__( self, app, *arg, **kw):
	self.cameraIndex = kw['camera']
	Plan.__init__(self, app, *arg, **kw ) # Initialize Plan
	
    # Gets the robot vision points
    def getState(self):
	state = self.vision.getState()
	return {'x':state[0],'y':state[1],'theta':state[2], 'tagX':state[3], 'tagY':state[4]}
    
    # Function to pass down waypoints to vision system
    def setWaypoints(self, waypoints):
	self.vision.setWaypoints(waypoints)
	
    # Function to pass down tag estimate position for rendering
    def setTagLocation(self, tagEst):
	pass#self.vision.setTagLocation(tagEst)
	
    def getTagLocationEstimate(self):
	return self.vision.tagLoc
    
    # Function to pass the force vector down for rendering
    def setControlVectorRender(self, start, end):
	self.vision.fVectorStart = start
	self.vision.fVectorEnd = end
	
    # Vision plan behavior
    def behavior( self ):
	#print 'Launching Vision Plan'
	if(self.cameraIndex):
	    print 'Starting vision system with camera', str(self.cameraIndex)
	    self.vision = VisionSystem(int(self.cameraIndex))
	else:
	    print 'Starting vision system with default camera'
	    self.vision = VisionSystem(0) # start vision system
	    
	# Main loop
	while(True):
	    self.vision.processFrame()
	    #yield self.forDuration(0.1)
	    if cv2.waitKey(5) & 0xFF == ord('q'):
		break
	    else:
		yield self.forDuration(0.05)
	    

	