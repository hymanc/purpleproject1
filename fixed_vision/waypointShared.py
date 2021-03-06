# file waypointShared.py contains definitions that are shared between the simulation
#   code in simTagStreamer and the waypointServer

## April tags associations ##########################################

from numpy import array, inf, asarray, uint8, empty
from numpy.random import randn
from pylab import plot, text
# Port for TagStreamer data
APRIL_DATA_PORT = 0xB00

# Host for waypointServer
#WAYPOINT_HOST = "127.0.0.1"
WAYPOINT_HOST = "10.0.0.1"

# Port for Waypoint messages
WAYPOINT_MSG_PORT = 0xBAA
WAYPOINT_LISTENERS = [ "224.0.1.133" ] + [ "10.0.0.%d" % h for h in xrange(2,12) ]

# Corners of the arena, in order
corners = [26,23,27,22,29,24,28,25]

# Tag ID for robot
ROBOT_TAGID = [ 4 ]

# Reference locations of corners, with 1 in the last coordinate
ref = array([
    [-1,0,1,1,1,0,-1,-1],
    [1,1,1,0,-1,-1,-1,0],
    [1.0/100]*8]).T * 100

# Tag IDs for waypoints
waypoints = range(4)

def lineSensorResponse( d, noise ):
  """
  Convert distances from line to line sensor measurements
  
  INPUT:
    d -- float(s) -- distance(s) from line (inf is allowed)
    noise -- float(s) -- scale of noise for measurements
    
    The shapes of d and noise must be the same, or noise should be
    scalar.

  OUTPUT: res
    for scalar inputs, res is an int in the range 0..255
    for array inputs, res is a uint8 array shaped like d+noise
  """
  d = abs(asarray(d).real)
  noise = asarray(noise)
  if noise.shape and (noise.shape != d.shape):
    raise ValueError("d.shape=%s is different from noise.shape=%s" % (str(d.shape),str(noise.shape)))
  res0 = 1/(1+d**2) + randn(*d.shape) * noise
  res1 = asarray(res0.clip(0,0.9999)*256, uint8)
  if res1.shape:
    return res1
  return int(res1)
  
def lineDist( c, a, b, scale=1.0 ):
  """
  Compute distance of point(s) c, to line segment from a to b
  
  INPUT:
    c -- complex array of any shape -- sensor locations
    a,b -- complex -- endpoints of line segment
  """
  # Rigid transform and rescaling that
  # takes (a,b) to (0,1), applied to c
  z = (c-a)/(b-a)
  far = (z.real<0) | (z.real>1)
  res = empty(z.shape,float)
  res[far] = inf
  d = z[~far].imag * abs(b-a)
  res[~far] = d/scale
  return res
  

class Sensor( object ):
  def __init__(self, *lineargs, **linekw):
    self.lineargs = lineargs
    self.linekw = linekw
    self.radius = None
    self.noise = 0.01

  def sense( self, a, b, c, scale=0.2 ):
    """Compute sensor measurement for line from a to b, given
       sensor location c and a scale factor
    """
    # Rigid transform and rescaling that
    # takes (a,b) to (0,1), applied to c
    z = (c-a)/(b-a)
    if (z.real<0) or (z.real>1):
      return lineSensorResponse( inf, self.noise )
    x = z.real * (b-a) + a
    d = z.imag * abs(b-a)
    res = lineSensorResponse( d/scale, self.noise )
    plot( [c.real, x.real], [c.imag, x.imag], 
      *self.lineargs, **self.linekw )
    text( (c.real+x.real)/2, (c.imag+x.imag)/2, 
      "%d" % res, ha='center',va='center' )
    return int(res)

