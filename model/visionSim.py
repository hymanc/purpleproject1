import cv2
import numpy as np
import math

# Camera Position is a 3-tuple relative to the field origin (center)


# Vision Simulation:
    # Compute pinhole camera model for world->image transform
    # Compute Static calibration with random error in correspondence points
    # At each step in simulation:
	# Compute quantized pixel coordinates of markers (off x-y plane) in raw simulated camera image from world coordinates
	# Transform marker image locations into rectified image
	# Add random noise to simulate uncertainty in HSV filter + Moment computation
    
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
	
	# Get tag x,y locations, convert to real space
	#self.computePerspectiveRectification()
	
    @staticmethod
    def defaultSim():
	tagLocations = [(-1,-1), (-1,1), (1,1), (1,-1)]
	return VisionSim((960,720), (0,0,0), 2E-3, tagLocations)
	
    # Generates the camera matrix
    def pinholeCameraMatrix(self, f):
	# Camera matrix = K*[R|t]
	# Compute y-rotation
	theta = -math.pi/2 # Camera Y-rotation
	tx = 0		# Camera X-translation
	ty = 0	# Camera Y-translation
	tz = 1	# Camera Z-translation
	fx = f/8.85E-6  # Focal length over pixel size (meters)
	fy = fx		# Square pixel assumption
	cx = (self.imageSize[0])/2
	cy = (self.imageSize[1])/2
	# Numpy matrix
	# fx 0  0  cx
	# 0  fy 0  cy
	# 0  0  1  1 
	K = np.array( [ [fx, 0., cx] , [0., fy, cy] , [0.,0.,1.] ] ) # Camera intrinsics
	#Rt = np.array( [ [1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.] ] )
	#print str(K)
	#print str(Rt)
	Rt = np.array( [ [1, 0, 0, tx] , [0,math.cos(theta),-math.sin(theta),ty] , [0,math.sin(theta),math.cos(theta),tz] ] ) # Only y-axis rotation of camera
	print str(Rt)
	self.camMatrix = np.dot(K,Rt) # Compute camera model (camMatrix)
	
    # Computes a perspective rectification 
    # worldCorr: World correspondence points (on the plane)
    # imageCorr: Image correspondence points (px coordinates)
    def computePerspectiveRectification(self, worldCorr, imageCorr):
	# Get 8 boundary centers
	# Add noise to each point (vary by <10px)
	# TODO: Remove OpenCV dependency
	for coord in imageCorr: # Add random noise and quantize image correspondences
	    coord = around(coord + np.random.normal(0,1,2),0)
	    # Add noise to each point
	self.perspectiveTf = cv2.getPerspectiveTransform(imageCorr, worldCorr) # Use opencv to compute LS solution
	return self.perspectiveTf
	
    # Convert points from worldCoordinates to image coordinates
    # worldCoordinates [3-tuples]
    # Return: [2-tuples]
    def computeImageCoordinates(self, worldCoordinates):
	iCoords = []
	i = 0
	for coord in worldCoordinates:
	    coord.append(1)
	    wc = np.array(coord)
	    wc = wc[np.newaxis, :].T
	    print 'WC:', str(wc)
	    print str(self.camMatrix)
	    imgPt = np.dot(self.camMatrix, wc)
	    imgPt = np.around(imgPt,0) # Quantize
	    iCoords.append(imgPt)
	    i = i + 1
	print str(iCoords)
	return iCoords
	# Compute image coordinate with quantization
	
    # Image to world estimate coordinate transform with quantization (no added noise)
    def computeImgToWorldEstimate(self, imgCoordinates):
	weCoords = []
	i = 0
	for coord in imgCoordinates:
	    estPt = np.dot(self.perspectiveTf, np.array(coord.append(1)).transpose())
	    estPt = np.around(estPt, 3) # Estimated quantization from remapping
	    weCoords[i] = estPt
	    i = i + 1
	return weCoords
    
    # Converts a world point into an estimated world point (w/ noise)
    def computeRemappedCoordinate(self, worldCoordinates, noiseSigma):
	imgCoords = computeImageCoordinates(worldCoordinates) # Compute and quantize image coordinates
	weCoords = computeImgToWorldEstimate(imgCoords) # Reproject using rectification transform and quantize
	for coord in weCoords: # Additive Gaussian noise
	    coord[0] = coord[0] + np.random.normal(0, noiseSigma, 1)
	    coord[1] = coord[1] + np.random.normal(0, noiseSigma, 1)
	return weCoords
    