import vision as v

class Localization(object):
	def __init__(self, VisionSystem):
		self.vis = VisionSystem
		self.pos = {'x':0, 'y':0, 't':0}