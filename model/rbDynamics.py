import numpy as np
from numpy.linalg import inv

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
	DynamicsPoint.__init__(center, forceVector)
	self.Torque = []
        self.T_dirMatrix = []
   	
    def computeTorque(self):
	
    	angle_si = np.arctan2([self.WP[1],  DynamicsPoint.getY], [self.WP[1],  DynamicsPoint.getX])
    	Fx = 10*np.cos(angle_si - self.theta)
    	Fy = 10*np.sin(angle_si - self.theta)
    	Force  = np.array([Fx], [Fy], [0]])
    	self.T_dirMatrix=np.array([[1., 0.5, -0.5], [0., -np.sqrt(3)/2, np.sqrt(3)/2], [0.1,0.1,0.1]])
    	invDmatrix = numpy.linalg.inv(self.T_dirMatrix)
    	self.Torque = invDmatrix*Force
    	
 

class displacement(PointForce):
	def __init__(self,theta, WP,Rcoorp, Rcoor):
		PointForce.__init__(center, forceVector,theta,WP)
		self.Rnext = []
	    
	def nextR(self):
		T = 0.1 #0.1 is the time increment
		m = ? #mass
		Rotation = np.array([[np.cos(self.theta), -1*np.sin(self.theta),0], [np.sin(self.theta), np.cos(self.theta),0],[0,0,1]])
		ForceNew = Rotation * self.T_dirMatrix * self.Torque
		Vp = (self.Rcoor - self.Rcoorp)/T 
		Vcurrent = Vp*0.5 + np.array([[ForceNew[0]*T/(2*m)], [ForceNew[1]*T/(2*m)]])
		self.Rnext = Rcoor+Vcurrent*T
		
	
"
Planar Rigid Body Dynamics Class based on NumPy/SciPy ODE
"
class RBDynamics(object):
    
    def __init__(self, masses):
	self.masses = masses
	self.forces = []
	self.torques = []
    
    def computeDynamicsStep(self):
	netForce = (0,0)
	for force in self.forces:
	    netForce = netForce + force.getValue()
	# Add all forces
	
	# Add all torques
	# Compute accelerations
	# Numerically Integrate to get positions
	
    # Center of Mass and mass
    def computeCoM(self):
	m0 = 0
	mx = 0
	my = 0
	# Compute 0 moments
	for m in self.masses:
	    m0 = m0 + getMass(m)
	    mx = mx + getMass(m) * getCenter(m)[0]
	return (mx, my), m0
	
    # Moment of inertia
    def computeI(self):
	
    # Instantaneous center of rotation
    def computeCoR(self):
    

	
