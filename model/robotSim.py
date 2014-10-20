# -*- coding: utf-8 -*-
"""
Created on Thu Sep  4 20:31:13 2014

@author: shrevzen-home
"""
from gzip import open as opengz
from json import dumps as json_dumps
from numpy import asfarray, dot, c_, newaxis, mean, exp, sum, sqrt
from numpy.linalg import svd
from numpy.linalg import inv
from numpy.random import randn
from waypointShared import *

from pdb import set_trace as DEBUG

import numpy as np
from math import *

from visionSim import VisionSim

MSG_TEMPLATE = {
       0: [[502, 251], [479, 272], [508, 296], [530, 274]],
       1: [[469, 347], [471, 311], [434, 305], [431, 339]],
       2: [[466, 226], [437, 209], [420, 231], [450, 248]],
       3: [[362, 204], [339, 224], [365, 245], [387, 224]],
       4: [[334, 336], [370, 326], [359, 294], [324, 301]],
       22: [[425, 145], [424, 169], [455, 173], [455, 149]],
       23: [[556, 302], [594, 308], [588, 277], [553, 272]],
       24: [[284, 258], [294, 229], [261, 221], [250, 249]],
       25: [[432, 424], [431, 384], [391, 380], [392, 419]],
       26: [[559, 399], [564, 440], [606, 444], [599, 403]],
       27: [[546, 163], [546, 187], [578, 193], [576, 170]],
       28: [[212, 397], [253, 395], [254, 355], [214, 358]],
       29: [[319, 129], [290, 124], [283, 145], [313, 153]]
  }
  
def tags2list( dic ):
    """ 
    Convert a dictionary of tags into part of a list for JSON serialization
    
    INPUT:
      dic -- dictionary mapping tag id to 4x2 corner location array
      
    OUTPUT:
      list to be concatenated into JSON message
    """
    return [ 
        { 
          u'i' : k, 
          u'p': [ list(row) for row in v ] 
        } 
        for k,v in dic.iteritems()
    ]

def findXing(a,b):
    """
    Find the crossing point of two lines, represented each by a pair of
    points on the line
    
    INPUT:
	a -- 2x2 -- two points, in rows
	b -- 2x2 -- two points, in rows
    
    OUTPUT: c -- 2 -- a point, as an array
    """
    a = asfarray(a)
    b = asfarray(b)
    # The nullspace of this matrix is the projective representation
    # of the intersection of the lines. Each column's nullspace is
    # one of the lines
    X = c_[a[1]-a[0],b[0]-b[1],a[0]-b[0]].T
    if X.ndim is not 2:
	DEBUG()
    Q = svd(X)[0]
    # Last singular vector is basis for nullspace; convert back from
    # projective to Cartesian representation
    q = Q[:2,2]/Q[2,2]
    c = q[0]*(a[1]-a[0])+a[0]
    return c

def randomFail(p):
    rn = np.random.rand(1)
    if rn > p:
	return 0
    return 1

def Checkslipping(motorspeed):
    """
    motor speed is a 4 by 1 vector
    identify which wheel is slipping
    """
    Identifymatix = (1/4) * np.matrix('1 1 -1 1; 1 1 1 -1;-1 1 1 1;1 -1 1 1')
    slip = Identifymatrix * motorspeed
  
    if slip[1] == 0 :
	slipwheel[0] = 0
    else:
	slipwheel[0] = 1
    
    if slip[2] == 0 :
	slipwheel[1] = 0
    else:
	slipwheel[1] = 1  
    
    if slip[3] == 0 :
	slipwheel[2] = 0
    else:
	slipwheel[2] = 1
    
    if slip[4] == 0 :
	slipwheel[3] = 0
    else:
	slipwheel[3] = 1
    return slipwheel
 
class RobotSimInterface( object ):
  """
  Abstract superclass RobotSimInterface defines the output-facing interface
  of a robot simulation.
  
  Subclasses of this class must implement all of the methods
  """
  def __init__(self, fn=None):
    """
    INPUT:
      fn -- filename / None -- laser log name to use for logging simulated 
          laser data. None logged if name is None
          
    ATTRIBUTES:
      tagPos -- 4x2 float array -- corners of robot tag
      laserAxis -- 2x2 float array -- two points along axis of laser
      waypoints -- dict -- maps waypoint tag numbers to 4x2 float 
          arrays of the tag corners
    """
    # Initialize dummy values into robot and arena state
    self.tagPos = asfarray(MSG_TEMPLATE[ ROBOT_TAGID[0]])
    self.laserAxis = dot([[1,1,0,0],[0,0,1,1]],self.tagPos)/2
    self.waypoints = { tid : asfarray(MSG_TEMPLATE[tid]) for tid in waypoints }
    ### Initialize internal variables
    # Two points on the laser screen
    self.laserScreen = asfarray([[-1,-1],[1,-1]])
    # Cache for simulated TagStreamer messages
    self._msg = None
    # Output for simulated laser data
    if not fn:
      self.out = None
    else:
      self.out = opengz(fn,"w")
    
  def refreshState( self ):
    """<<pure>> refresh the value of self.tagPos and self.laserAxis"""
    print "<<< MUST IMPLEMENT THIS METHOD >>>"
    
  def getTagMsg( self ):
    """
    Using the current state, generate a TagStreamer message simulating
    the robot state
    """
    # Cache a list of the corner tags. They don't move
    if self._msg is None:
      self._msg = tags2list({ tid : MSG_TEMPLATE[tid] for tid in corners})
    # Collect robot and waypoint locations
    state = { ROBOT_TAGID[0] : self.tagPos }
    state.update( self.waypoints )
    # Combine all into a list
    msg = tags2list(state) + self._msg
    # Serialize
    return json_dumps(msg)

  def logLaserValue( self, now ):
    """
    Using the current state, generate a fictitious laser pointer reading
    INPUT:
      now -- float -- timestamp to include in the message
    
    OUTPUT: string of human readable message (not what is in log)      
    """
    x = findXing( self.laserScreen, self.laserAxis )
    if self.out:
	self.out.write("%.2f, 1, %d, %d\n" % (now,n+1,x[0],x[1]))          
    return "Laser: %d,%d " % tuple(x)
  
class WheelUncertainties(object):
    """
    if the torque applied on the wheel is higher than the traction
    then the wheel will slip
    here we make wheel 1 slip
    """
    def __init__(self,torque):
	self.makeWheel1slip(torque)
        
    def makeWheel1slip(self,torque): 
	self.traction = np.random.normal(10,0.1,1) # Changed param 3 to 1 from 1000
	while (torque < self.traction):
	    self.traction = np.random.normal(10,0.1,1)
	return self.traction



# Abstract Container class for planar dynamics types with a center and value
class DynamicsPoint(object):
    def __init__(self, center, value):
	self.center = center
	self.value = value
    
    def getCenter(self):
	return center
    
    def getX(self):
	return self.center[0]
    
    def getY(self):
	return self.center[1]
    
    def getValue(self):
	return self.value
    
# Point Mass class
# x,y position of point mass (x)
# Mass value (m)
class PointMass(DynamicsPoint):
    def __init__(self,center,m):
	DynamicsPoint.__init__(center, m)

# Moment of inertia class 
# x,y position of axis
# Moment value
class MomentOfInertia(DynamicsPoint):
    def __init__(self, center, I):
	DynamicsPoint.__init__(center, I)
	
# Force Class
class PointForce(DynamicsPoint):
    def __init__(self, center, forceVector,theta,WP):
	DynamicsPoint.__init__(self, center, forceVector)
	self.WP = WP
	self.theta = theta
	self.Torque = []
	self.slipTorque = []
	self.T_dirMatrix = []
	self.computeTorque()
	self.slipDisplacement()
	self.dNoise = 0.1
   	
    def computeTorque(self):
    	angle_si = atan2(self.WP[1] - self.getY(), self.WP[1] - self.getX())
    	Fx = 10*np.cos(angle_si - self.theta)
    	Fy = 10*np.sin(angle_si - self.theta)
    	Force  = np.array([[Fx], [Fy], [0]])
    	self.T_dirMatrix=np.array([[1., 0.5, -0.5], [0., -np.sqrt(3)/2, np.sqrt(3)/2], [0.1,0.1,0.1]])
    	invDmatrix = np.linalg.inv(self.T_dirMatrix)
    	self.Torque = invDmatrix*Force
    
    def slipDisplacement(self):
    	self.slipTorque = np.array([[(1+randn()*self.dNoise), 0, 0],[0,(1+randn()*self.dNoise), 0],[0,0,(1+randn()*self.dNoise)]])*self.Torque
 	

class Displacement(PointForce):
    def __init__(self, theta, WP, Rcoorp, Rcoor):
	forceVector = np.array(Rcoor)-np.array(WP)
	PointForce.__init__(self, Rcoor, forceVector, theta, WP)
	self.Rnext = []
	self.nextR()
		
    def nextR(self):
	T = 0.1 #0.1 is the time increment
	m = 0.7 #mass
	Rotation = np.array([[np.cos(self.theta), -1*np.sin(self.theta),0], [np.sin(self.theta), np.cos(self.theta),0],[0,0,1]])
	ForceNew = Rotation * self.T_dirMatrix * self.Torque
	Vp = (self.Rcoor - self.Rcoorp)/T 
	Vcurrent = Vp*0.5 + np.array([[ForceNew[0]*T/(2*m)], [ForceNew[1]*T/(2*m)]])
	self.Rnext = Rcoor+Vcurrent*T
	
class SlipDisplacement(PointForce):
    def __init__(self,theta, WP,Rcoorp, Rcoor):
	PointForce.__init__(center, forceVector,theta,WP)
	self.slipRnext = []
	self.slipnextR()
		
    def slipnextR(self):
	T = 0.1 #0.1 is the time increment
	m = 0.7 #mass
	Rotation = np.array([[np.cos(self.theta), -1*np.sin(self.theta),0], [np.sin(self.theta), np.cos(self.theta),0],[0,0,1]])
	slipForceNew = Rotation * self.T_dirMatrix * self.slipTorque
	Vp = (self.Rcoor - self.Rcoorp)/T 
	Vcurrent = Vp*0.5 + np.array([[ForceNew[0]*T/(2*m)], [ForceNew[1]*T/(2*m)]])
	self.slipRnext = Rcoor+Vcurrent*T
	    
		
class DummyRobotSim( RobotSimInterface, WheelUncertainties, VisionSim ):
    def __init__(self, *args, **kw):
	RobotSimInterface.__init__(self, *args, **kw)
	tagLocations = [[-1,0,-1], [-1,0,1], [1,0,1], [1,0,-1]]
	defaultHeight = 0.09 #Approx 3.5" for tag height
	focalLength = 2E-3
	VisionSim((960,720), (0,1,-1), pi/2, focalLength, tagLocations, defaultHeight)	
	#WheelUncertainties.__init__(self, torque)
	#Displacement(self,theta, WP,Rcoorp, Rcoor)
	self.dNoise = 0.1
	self.previousPosition = [[0],[0]]
	self.laserAngle = 0
	#slip = Checkslipping(motorspeed) # motorspeed is 4x1 vector
    
    def computeStep(self, waypoint):
	print 'Computing step'
	markerPoints = self.getRealMarkerPoints()# Get actual marker points
	centerEst, thetaEst, phiEst = self.estimateState(markerPoints)
	# Get reprojected markers
	# Compute center, angles from markers
	# Compute laser error and generate feedback
	rn = randomFail(0.01)
	if(rn == 1):
	    d = SlipDisplacement(thetaEst, waypoint, self.previousPosition, centerEst)
	else:
	    d = Displacement(thetaEst, waypoint, self.previousPosition, centerEst)
	    
	self.previousPositoin = centerEst
	    
    def move( self ):
	# Move all tag corners forward to new R-coordinates, with some noise
	self.tagPos = self.tagPos + (dot(np.array[[1],[1],[1],[1]],np.transpose(self.Rnex)  * (1+randn()*self.dNoise)))

	
    # Gets the sticker marker points
    def getRealMarkerPoints(self):
	print str(self.tagPos)
	return (1,0,0),(0,0,1),(1,0,1)
	
    def slipmove( self ):
    	self.tagPos = self.tagPos + (dot(np.array[[1],[1],[1],[1]],np.transpose(self.slipRnex)  * (1+randn()*self.dNoise)))
    	
    
    def refreshState( self ):
	"""
	Make state ready for use by client.
	
	ALGORITHM:
	Since the entire robot state is captured by the location of the
	robot tag corners, update the laser axis from the robot tag location 
	"""
	self.laserAxis = dot([[1,1,0,0],[0,0,1,1]],self.tagPos)/2
	da = dot([1,-1],self.laserAxis)
	self.laserAxis[1] += randn(2) * sqrt(sum(da*da)) * 0.01
	
	
    
    
