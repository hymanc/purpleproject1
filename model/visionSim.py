import cv2
import numpy as np

# Camera Position is a 3-tuple relative to the field origin (center)

class VisionSim(object):
    
    #imageSize: 2-tuple; image size (width, height)
    #cameraPosition: 3-tuple; Camera origin wrt world center (assume camera is pointed at center)
    #cameraF: floating point number; Camera focal length (meters)
    #tagLocations: Boundary tag world locations (meters)
    def __init__(self, imageSize, cameraPosition, cameraF, tagLocations):
	self.imageSize = imageSize
	self.cameraPosition = cameraPosition
	self.tagLocations = tagLocations
	self.pinholeCameraMatrix(cameraF)
	
    # Calibration
    # World Objects -> Camera perspective
    # Tag Locations in camera frame -> Perspective Calibration
    # Apply perspective calibration to marker points
    # Add 
    
    # Compute Combined camera/parallax/rectification model
    # For each step
	# Compute actual marker centers
	# Apply quantization and random noise with low variance (experimental values?)
	# Apply position filter
	# Compute error to actual positions
    
    # Generates the camera matrix
    def pinholeCameraMatrix(self, f):
	# Camera matrix = K*[R|t]
	# Compute y-rotation
	theta = 0
	tx = 0
	ty = 0
	# Numpy matrix
	# fx 0  0  cx
	# 0  fy 0  cy
	# 0  0  1  1 
	K = np.array([fx, 0., cx],[0., fy, cy],[0.,0.,1.])
	Rt = np.array([cos(theta),0,sin(theta),tx],[0,1,0,ty],[-sin(theta),0,cos(theta),1.]) # Only y-axis rotation
	self.camMatrix = np.dot(K,Rt)
	
    # Computes a perspective rectification 
    # worldCorr: World correspondence points (on the plane)
    # imageCorr: Image correspondence points (px coordinates)
    def computePerspectiveRectification(self, worldCorr, imageCorr):
	
	# Get 8 boundary centers
	# Add noise to each point (vary by <10px)
	# Use opencv to compute LS solution
	
    # Convert points from worldCoordinates to image coordinates
    # worldCoordinates [3-tuples]
    # Return: [2-tuples]
    def computeImageCoordinates(self, worldCoordinates):
	iCoords = []
	i = 0
	for coord in worldCoordinates:
	    imgPt = np.dot(self.CamMatrix, np.array(coord).transpose())
	    imgPt = np.around(imgPt) # Quantize
	    iCoords[i] = imgPt
	    i = i + 1
	return iCoords
	# Compute image coordinate with quantization
	
    def computeImgToWorldEstimate(self, imgCoordinates):
	weCoords = []
	i = 0
	for coord in imgCoordinates:
	    estPt = np.dot(self.prectMatrix, np.array(coord.append(1)).transpose())
	    estPt = np.around(estPt, 3) # Estimated quantization from remapping
	    weCoords[i] = estPt
	    i = i + 1
	return weCoords
    
    # Converts a world point into an estimated world point (w/ noise)
    def computeRemappedCoordinate(self, worldCoordinates, noiseSigma):
	imgCoords = computeImageCoordinates(worldCoordinates) # Compute and quantize image coordinates
	weCoords = computeImgToWorldEstimate(imgCoords) # Reproject using rectification transform and quantize
	for coord in weCoords:
	    coord[0] = coord[0] + np.random.normal(0, noiseSigma, 1) # Add noise
	    coord[1] = coord[1] + np.random.normal(0, noiseSigma, 1)
	return weCoords
    