import os
import sys
sys.path.insert(0, os.getcwd())

from joy import JoyApp
from fixed_vision.visionPlan import VisionPlan

class DoritoApp( JoyApp ):
    
    def __init__(self):
	JoyApp.__init__( self, confPath="$/cfg/JoyApp.yml") 

	
    def onStart( self ):
	print 'Doritos Xtreme Command and Control v99.37'
	self.vplan = VisionPlan(self)
	self.vplan.start()
	
	
    def onEvent( self ):
	print 'Event!'
	
if __name__=="__main__":
  import sys
  if len(sys.argv)>1:
      app=DoritoApp()
  else:
      app=DoritoApp()
  app.run()
	
	
	