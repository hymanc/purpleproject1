
from visionSim import VisionSim
import numpy as np

vsim = VisionSim.defaultSim()

corners = [ [-1,0,-1], [-1,0,1], [1,0,1], [1,0,-1] ]

icoords = vsim.computeImageCoordinates(corners)
weCoords = vsim.computeImageToWorldEstimate(icoords)

for coord in weCoords:
    print 'Result', coord.flatten()