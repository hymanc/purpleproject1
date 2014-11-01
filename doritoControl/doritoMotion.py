import numpy as np

from math import *
import ckbot.logical #need to make sure modules can populate and 
from joy.decl import *

# Dorito Robot Motion Class
class DoritoMotion(object):
    
    # Class constructor
    def __init__(self):
	# Enumerate servos
	self.c = ckbot.logical.Cluster()
	self.c.populate(3,{ 0x03 : 'D0', 0x04 : 'D1', 0x0B : 'D2'})
	self.theta = 0	# Rotation state needed for command generation
	#TODO: Enumeration error checking?
	
    # Sets the desired linear and angular velocity
    # Use this function to move the robot
    # lVelocity: Linear velocity setpoint, 2-tuple
    # aVelocity: Angular velocity setpoint, scalar
    # theta: Current robot rotation for correct force direction
    def setSpeed(self, lVelocity, aVelocity, theta=None):
	if(theta != None):
	    self.theta = theta
	    
	self.setDriveMotorCommands(self._generateMotorCommands(lVelocity, aVelocity))
	
    # Sets the drive motor torque commands to the specified values
    # Commands should be a 3-list or 3-tuple
    # Individual values should be between -1 and 1 or they will not be set
    def setDriveMotorCommands(self, commands):
	self.c.at.D0.set_torque(commands[0])
	self.c.at.D1.set_torque(commands[1])
	self.c.at.D2.set_torque(commands[2])

    # Sets an individual motor torque command
    def _setMotorCommand(self, motor, command):
	if(abs(command) > 1): # Normalize if too big
	    command = command/abs(command) 
	if(abs(command) < 0.01):
	    motor.go_slack() # Go slack if values small enough
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
    
    # Generates the 3x3 Force-Torque matrix rotated into the world frame
    def _forceTorqueMatrix(self, worldFrameFlag=True,theta=None):
	if(theta == None):
	    theta = self.theta
	rc = cos(theta)
	rs = sin(theta)
	Fx1 = -cos(pi/3) 
	Fx2 = 1
	Fx3 = -cos(pi/3)
	Fy1 = -sin(pi/3) 
	Fy2 = 0
	Fy3 = sin(pi/3)
	F = np.array([ [Fx1,Fx2,Fx3] , [Fy1,Fy2,Fy3], [-1,-1,-1] ])
	if(worldFrameFlag): # Rotate forces into world frame
	    R = np.array([[rc,-rs,0],[rs,rc,0],[0,0,1]])
	    return np.dot(R,F)
	else:
	   return F
    
    # Current rotation state mutator
    def setTheta(self, theta):
	self.theta = theta
	
    # Current rotation state accessor
    def getTheta(self):
	return self.theta