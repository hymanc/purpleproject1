import numpy as np

from joy import JoyApp
from math import *
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
	self.watchdogTick = 2
	self.c = ckbot.logical.Cluster()
	self.c.populate(4,{ 0x03 : 'D0', 0x04 : 'D1', 0x0B : 'D2', 0x08 : 'L0'} )
	#for i in range(0,self.nservos):
	    #self.c.populate() # node IDs of modules are stored
	self.state = (0.,0., pi/2,0.)					# State (x,y,theta,phi)
	
	#TODO: Check for motor enumeration
	#self.dMot = [self.c.at.Nx03, self.c.at.Nx04, self.c.at.Nx0B]	# Drive motors
	#self.lMot = self.c.at.Nx08					# Laser Turret
	self.watchdogTime = self.onceEvery(4)
	self.controls = {'up':False, 'down':False, 'left':False, 'right':False, 'cw':False, 'ccw':False}
		 
	
    # Sets the desired linear and angular velocity
    # Use this function to move the robot
    # lVelocity: Linear velocity setpoint, 2-tuple
    # aVelocity: Angular velocity setpoint, scalar
    def setSpeed(self, lVelocity, aVelocity):
	self.setDriveMotorCommands(self._generateMotorCommands(lVelocity, aVelocity))
	
    #
    def setDriveMotorCommands(self, commands):
	self.c.at.D0.set_torque(commands[0])
	self.c.at.D1.set_torque(commands[1])
	self.c.at.D2.set_torque(commands[2])

    # Sets a motor torque command
    def _setMotorCommand(self, motor, command):
	if(abs(command) < 0.05):
	    motor.go_slack()
	else:
	    motor.set_torque(command)
	
    # Generates desired torque commands
    # Force is a 2-tuple
    # Torque is a scalar
    def _generateMotorCommands(self, force, torque):
	ftVector = np.array([ [force[0]], [force[1]], [torque] ]) # Force torque vector
	rfi = np.linalg.inv(self._forceTorqueMatrix(True)) # System matrix
	cmds = np.dot(rfi, ftVector).reshape(3) # Motor commands as a list
	if(max(abs(cmds)) > 1):
	    print 'Saturating commands'
	    cmds = cmds/(max(abs(cmds))) # Keep directions but limit power
	print 'Motor Commands', str(cmds)
	return np.asarray(cmds) # Return commands as array
    
    # Generates the 3x3 Force-Torque matrix rotated into the 
    def _forceTorqueMatrix(self, worldFrameFlag):
	rc = cos(self.getTheta())
	rs = sin(self.getTheta())
	Fx1 = -cos(pi/3) 
	Fx2 = 1
	Fx3 = -cos(pi/3)
	Fy1 = -sin(pi/3) 
	Fy2 = 0
	Fy3 = sin(pi/3)
	F = np.array([ [Fx1,Fx2,Fx3] , [Fy1,Fy2,Fy3], [-1,-1,-1] ])
	if(worldFrameFlag): #TODO: Actually update state
	    R = np.array([[rc,-rs,0],[rs,rc,0],[0,0,1]])
	    return np.dot(R,F)
	else:
	   return F

    def _parseControls(self):
	f = np.array((0.,0.))
	t = 0.
	print 'Controls: ', self.controls
	if(self.controls['up']):
	    f = f + np.array((0., 3.))
	if(self.controls['down']):
	    f = f + np.array((0., -3.))
	if(self.controls['left']):
	    f = f + np.array((-3., 0.))
	if(self.controls['right']):
	    f = f + np.array((3., 0.))
	if(self.controls['ccw']):
	    t = t + 3.
	if(self.controls['cw']):
	    t = t - 3.
	print 'Controls parsed',np.asfarray(f),t
	self.setSpeed(np.asarray(f), t)
	    
	
    # Sets state vector
    def setState(self, state):
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
    
        if self.watchdogTime(): 
	    self.watchdogTick = self.watchdogTick - 1
	    if(self.watchdogTick == 0):
		self._setMotorCommand(self.c.at.D0,0)
		self._setMotorCommand(self.c.at.D1,0)
		self._setMotorCommand(self.c.at.D2,0)
	    # TODO: watchdog
	    
	if evt.type == KEYUP:
	    self.watchdogTick = 2
	    if evt.key == K_UP:
		self.controls['up'] = False
		#self.setSpeed((0,1),0)
	    elif evt.key == K_DOWN:
		self.controls['down'] = False
		#self.setSpeed((0,-1),0)
	    elif evt.key == K_LEFT:
		self.controls['left'] = False
		#self.setSpeed((-1,0),0)
	    elif evt.key == K_RIGHT:
		self.controls['right'] = False
		#self.setSpeed((1,0),0)
	    elif evt.key == K_PAGEUP:
		self.controls['ccw'] = False
		#self.setSpeed((0,0),1)
	    elif evt.key == K_PAGEDOWN:
		self.controls['cw'] = False
		#self.setSpeed((0,0),-1)
	    self._parseControls()
	    '''
	    if evt.key == K_UP or \
	    evt.key == K_DOWN or \
	    evt.key == K_LEFT or \
	    evt.key == K_RIGHT or \
	    evt.key == K_PAGEUP or \
	    evt.key == K_PAGEDOWN:
		self.setDriveMotorCommands([0,0,0])
	    '''

	if evt.type == KEYDOWN:
	    self.watchdogTick = 2
	   
	    if evt.key == K_UP:
		self.controls['up'] = True
		#self.setSpeed((0,1),0)
	    elif evt.key == K_DOWN:
		self.controls['down'] = True
		#self.setSpeed((0,-1),0)
	    elif evt.key == K_LEFT:
		self.controls['left'] = True
		#self.setSpeed((-1,0),0)
	    elif evt.key == K_RIGHT:
		self.controls['right'] = True
		#self.setSpeed((1,0),0)
	    elif evt.key == K_PAGEUP:
		self.controls['ccw'] = True
		#self.setSpeed((0,0),1)
	    elif evt.key == K_PAGEDOWN:
		self.controls['cw'] = True
		#self.setSpeed((0,0),-1)
	    self._parseControls()
	    
	
	
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