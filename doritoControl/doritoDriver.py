import numpy as np

from joy import JoyApp
import math
import ckbot.logical #need to make sure modules can populate and the pcan can recognise the node ID
import joy  #used to call the feedbackrate function
import time #used for all of the time related functions
import pylab   
from joy.decl import *

# Dorito Robot Motion Class
class DoritoDriver ( JoyApp ):
    def __init__(self):
	JoyApp.__init__( self, confPath="$/cfg/JoyApp.yml",)
	#self.c = ckbot.logical.Cluster() #program recognises how many and which modules are attached
	self.nservos = 4
	self.c = ckbot.logical.Cluster()
	for i in range(0,self.nservos):
	    self.c.populate() # node IDs of modules are stored
	self.state = (0.,0.,0.,0.)					# State (x,y,theta,phi)
	self.dMot = [self.c.at.Nx03, self.c.at.Nx04, self.c.at.Nx0B]	# Drive motors
	self.lMot = self.c.at.Nx08					# Laser Turret
	self.timeForKeyClear = self.onceEvery(0.1)
	
    # Sets a motor torque command
    def setMotorCommand(self, motor, command):
	if(abs(command) < 0.05):
	    motor.go_slack()
	else:
	    motor.set_torque(command)
	
    # Generates desired torque commands
    def generateMotorCommands(self, force, torque):
	# Force/torque vector
	rfi = np.inv(self.forceTorqueMatrix(self.getTheta)) # 
	# Solve for direction
    
    # Generates 
    def forceTorqueMatrix(self, theta):
	rc = cos(self.state[t])
	rs = sin(self.state[t])
	Fx1 = cos(math.pi/3) #TODO: Change force directions
	Fx2 = -1
	Fx3 = -cos(math.pi/3)
	Fy1 = sin(math.pi/3) 
	Fy2 = 0
	Fy3 = -sin(math.pi/3)
	R = np.array([[rc,rs,0],[-rs,rc,0],[0,0,1]])
	F = np.array([ [Fx1,Fx2,Fx3] , [Fy1,Fy2,Fy3], [1,1,1] ])
	return np.dot(R,F) #Planar Force/Torque translation matrix in world frame
	
    # Sets state vector
    def setState(self, state):
	# TODO: Vector to dict
	self.state = state
	
    def setX(self, x):
	self.state[0] = x
	
    def setY(self, y):
	self.state[1] = y
	
    def setTheta(self, theta):
	self.state[2] = theta
	
    def setPhi(self, phi):
	self.state[3] = phi
	
    # State accessors
    def getState(self):
	return self.state
	
    def getX(self):
	return self.state[0]
    
    def getY(self):
	return self.state[1]
    
    def getTheta(self):
	return self.state[2]
    
    def getPhi(self):
	return self.state[3]
    
    def onEvent( self, evt ):
    
        if self.timeForKeyClear(): 
	    keyclear = 0 # Do nothing
	    
	if evt.type == KEYUP:
	    if evt.key == K_a:
		self.setMotorCommand(self.dMot[0],0)
	    elif evt.key == K_z:
		self.setMotorCommand(self.dMot[0],0)
	    elif evt.key == K_f:
		self.setMotorCommand(self.dMot[1],0)
	    elif evt.key == K_v:
		self.setMotorCommand(self.dMot[1],0)
	    elif evt.key == K_j:
		self.setMotorCommand(self.dMot[2],0)
	    elif evt.key == K_m:
		self.setMotorCommand(self.dMot[2],0)

	if evt.type == KEYDOWN:
	    if evt.key == K_a:
		print 'Q'
		self.setMotorCommand(self.dMot[0],0.8)
	    elif evt.key == K_z:
		print 'Z'
		self.setMotorCommand(self.dMot[0],-0.8)
	    elif evt.key == K_j:
		print 'J'
		self.setMotorCommand(self.dMot[2],0.8)
	    elif evt.key == K_m:
		self.setMotorCommand(self.dMot[2],-0.8)
	    elif evt.key == K_f:
		self.setMotorCommand(self.dMot[1],0.8)
	    elif evt.key == K_v:
		self.setMotorCommand(self.dMot[1],-0.8)

	# Use superclass to show any other events
	return JoyApp.onEvent(self,evt)
    

if __name__=="__main__":
  print """
  Running the robot simulator

  Listens on local port 0xBAA (2986) for incoming waypointServer
  information, and also transmits simulated tagStreamer messages to
  the waypointServer. 
  """
  import sys
  app=DoritoDriver()
  #if len(sys.argv)>1:
      #app=DoritoDriver(wphAddr=sys.argv[1])
  #else:
      #app=DoritoDriver(wphAddr=WAYPOINT_HOST)
  app.run()