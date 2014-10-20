
from visionSim import VisionSim
import numpy as np
from math import *


#vsim = VisionSim.defaultSim()

tagLocations = [[-100,0,-100], [-100,0,100], [100,0,100], [100,0,-100]]
defaultHeight = 0.09 #Approx 3.5" for tag height
focalLength = 100*2E-3
vsim = VisionSim((960,720), (0,100,-150), 0.4*pi, focalLength, tagLocations, defaultHeight)


corners = [ [-100,0,-100], [-100,0,100], [100,0,100], [100,0,-100] ]

icoords = vsim.computeImageCoordinates(corners)
weCoords = vsim.computeImageToWorldEstimate(icoords)
itcoord = [-100,0,100]
tcoord = vsim.computeRemappedCoordinate([itcoord], 0.00001)
for coord in weCoords:
    print 'Result', coord

out = (float(tcoord[0][0]),float(tcoord[0][1]))
invalue = (float(itcoord[0]),float(itcoord[2]))
print 'Input:',str(invalue)
print 'Output:',str(out)
error = np.linalg.norm(np.array(out) - np.array(invalue))
print 'Error:',str(error)