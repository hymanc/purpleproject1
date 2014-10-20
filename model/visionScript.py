
from visionSim import VisionSim
import numpy as np
from math import *


#vsim = VisionSim.defaultSim()

tagLocations = [[-1,0,-1], [-1,0,1], [1,0,1], [1,0,-1]]
defaultHeight = 0.09 #Approx 3.5" for tag height
focalLength = 2E-3
vsim = VisionSim((960,720), (0,1.5,1.5), pi/4, focalLength, tagLocations, defaultHeight)


corners = [ [-1,0,-1], [-1,0,1], [1,0,1], [1,0,-1] ]

icoords = vsim.computeImageCoordinates(corners)
weCoords = vsim.computeImageToWorldEstimate(icoords)
itcoord = [-1,0,-1]
tcoord = vsim.computeRemappedCoordinate([itcoord], 0.00001)
for coord in weCoords:
    print 'Result', coord

out = (float(tcoord[0][0]),float(tcoord[0][1]))
invalue = (float(itcoord[0]),float(itcoord[2]))
print 'Input:',str(invalue)
print 'Output:',str(out)
error = np.linalg.norm(np.array(out) - np.array(invalue))
print 'Error:',str(error)