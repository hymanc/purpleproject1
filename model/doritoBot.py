class DoritoBot( RobotSimInterface ):
    def __init__(self, *args, **kw):
	RobotSimInterface.__init__(self, *args, **kw)
	self.servoMass = 
    # Translate at a given speed and direction
    def translate(self, speed, angle):
	# Compute new tag position
	
    # Rotate to desired angle at a given angular speed
    def rotate(self, speed, angle):
	#
    
    # Combined translate and rotate
    def transrotate(self, speed, tAngle, rAngle):
	#
	
    def refreshState(self):
	self.laserAxis =
	da =
	
    def computeDynamics(self):
	# Compute current wheel torques
	# Apply friction model to get forces at axles
	# Do sum of forces
	# Do sum of torques