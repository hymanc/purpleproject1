import numpy as np

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
    def __init__(self, center, forceVector):
	DynamicsPoint.__init__(center, forceVector)
	
    def computeTorque(self, axis):
	# Implement r x F in 2D
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
	