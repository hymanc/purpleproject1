from visionPlan import VisionPlan  # Vision System Plan
from sensorPlan import SensorPlan  # Sensor System Plan
from waypointShared import *	# Waypoint shared definitions

# Combined Feedback Class for Doritobot
# Combines vision feedback with commands from 
class DoritoFeedbackPlan( Plan ):
    
    def __init__(self):
	self.vis = VisionPlan(self, 1) # Initialize Vision System
	self.state = {'x':0,'y':0,'theta':0}
	# Sensor Plan for Targets
	self.sensor = SensorPlan(self)
	self.sensor.start()
	
    # Get estimate from vision plan
    def estimateState(self):
	self.state = # Get updated state from vision system
	return self.state
    
    # Gets next waypoint from sensor plan
    def getNextWaypoint(self):
	if(len(self.sensor.lastWaypoints[1])):
	    self.nextWaypoint = self.sensor.lastWaypoints[1][0]# Get next waypoint from sensorplan
	else:
	    print 'No Remaining Waypoints, WOOT!'
	    self.nextWaypoint = None # Empty waypoint
	return self.nextWaypoint 
    
    
    
	#10.0.0.98 pw:viewonly
