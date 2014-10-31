def _parseControlsv2(self,nextwp_loc):
	drivescale = 1.
	rotscale = 1.
	curX = self.getX()
	curY = self.getY()
	offA = self.getTheta()
	curloc=np.array((curX,curY))
	f = drivescale*(nextwp_loc-curloc)
	t = rotscale*(-offA)
	
	print 'Controls v2 parsed',np.asfarray(f),t
	self.setSpeed(np.asarray(f), t)
	    