import cv2
import numpy as np
from math import *

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
    def __init__(self, imageSize, cameraPosition, cameraTilt, cameraF, tagLocations, robotHeight):
	self.imageSize = imageSize
	self.cameraPosition = cameraPosition
	self.cameraTilt = cameraTilt
	self.tagLocations = tagLocations
	self.cameraToWorldRigid()
	self.worldToCameraRigid()
	self.pinholeCameraMatrix(cameraF)
	self.robotHeight = robotHeight
	imTagCoord = self.computeImageCoordinates(tagLocations)
	self.computePerspectiveRectification(tagLocations, imTagCoord)
	# Get tag x,y locations, convert to real space
	#self.computePerspectiveRectification()
	
    @staticmethod
    def defaultSim():
	tagLocations = [[-1,0,-1], [-1,0,1], [1,0,1], [1,0,-1]]
	defaultHeight = 0.09 #Approx 3.5" for tag height
	focalLength = 2E-3
	return VisionSim((960,720), (0,1,1), pi/4, focalLength, tagLocations, defaultHeight)
	
    # Rigid transform between world frame and camera frame
    def worldToCameraRigid(self):
	t = self.cameraTilt
	#self.wtc = [
	#[1,0,0,-self.cameraPosition[0]],
	#[0,cos(t),sin(t),-self.cameraPosition[1]],
	#[0,-sin(t),cos(t),-self.cameraPosition[2]],
	#[0,0,0,1]]
	self.wtc = np.linalg.inv(self.cameraToWorldRigid())
	return self.wtc
    
    # Rigid transform between camera frame and world frame
    def cameraToWorldRigid(self):
	#self.ctw = np.linalg.inv(self.worldToCameraRigid())
	t = self.cameraTilt
	self.ctw = [
	[1,0,0,self.cameraPosition[0]],
	[0,cos(t),-sin(t),self.cameraPosition[1]],
	[0,sin(t),cos(t),self.cameraPosition[2]],
	[0,0,0,1]]
	return self.ctw
    
    # Converts a point in the world rigidframe to the camera rigid frame
    def ptToCamera(self, point):
	point = np.array(point)
	point = point[np.newaxis,:].T
	cpoint = np.dot(self.wtc, point)
	return cpoint
	
    # Converts a point in the camera rigid frame to the world rigid frame
    def ptToWorld(self, point):
	point = np.array(point)
	point = point[np.newaxis,:].T
	wpoint = np.dot(self.ctw, point)
	return wpoint
	
    # Generates the camera matrix
    def pinholeCameraMatrix(self, f):
	# Camera matrix = K*[R|t]
	# Compute y-rotation
	#theta = self.cameraTilt # Camera Y-rotation
	tx = self.cameraPosition[0]	# Camera X-translation
	ty = self.cameraPosition[1]	# Camera Y-translation
	tz = self.cameraPosition[2]	# Camera Z-translation
	fx = f/8.85E-6  # Focal length over pixel size (meters)
	fy = fx		# Square pixel assumption
	cx = (self.imageSize[0])/2
	cy = (self.imageSize[1])/2
	cx = 0
	cy = 0
	# Numpy matrix
	# fx 0  0  cx
	# 0  fy 0  cy
	# 0  0  1  1 
	self.K = np.array( [ [fx, 0., cx] , [0., fy, cy] , [0.,0.,1.] ] ) # Camera intrinsics
	self.Rt = np.array( [ [1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.] ] )
	#self.Rt = np.array( [ [1, 0, 0, tx] , [0,cos(theta),-sin(theta),ty] , [0,sin(theta),cos(theta),tz] ] ) # Only y-axis rotation of camera
	self.camMatrix = np.dot(self.K,self.Rt) # Compute camera model (camMatrix)
	
    # Computes a perspective rectification 
    # worldCorr: World correspondence points (on the plane)
    # imageCorr: Image correspondence points (px coordinates)
    def computePerspectiveRectification(self, worldCorr, imageCorr):
	# Get 8 boundary centers
	# Add noise to each point (vary by <10px)
	A = []
	b = []
	i = 0
	for coord in imageCorr: # Add random noise and quantize image correspondences
	    #coord = np.around(coord + np.random.normal(0,1,2),0) # Add noise to each point
	    wc = self.ptToCamera(worldCorr[i])
	    #wc = np.array(worldCorr[i])
	    #wc = wc[np.newaxis,:].T
	    #wc = np.dot(self.wtc, wc)
	    #Xw = worldCorr[i][0]
	    #Yw = worldCorr[i][2]
	    Xw = float(wc[0])
	    Yw = float(wc[1])
	    xi = np.around(coord[0] + np.random.normal(0,1,1),0)[0]
	    yi = np.around(coord[1] + np.random.normal(0,1,1),0)[0]
	    an1 = [xi,yi,1.,0.,0.,0.,-Xw*xi,-Xw*yi] # Add correspondences to matrix
	    an2 = [0.,0.,0.,xi,yi,1.,-Yw*xi,-Yw*yi]
	    bn1 = [Xw]
	    bn2 = [Yw]
	    A.append(an1)
	    A.append(an2)
	    b.append(bn1)
	    b.append(bn2)
	    i = i + 1
	#self.perspectiveTf = cv2.getPerspectiveTransform(imageCorr, worldCorr) # Old opencv dependency
	# Least squares solution Ax=B
	A = np.array(A)
	x = np.linalg.solve(A,b)
	r1 = x[0:3]
	r2 = x[3:6]
	r3 = x[6:8]
	r3 = np.append(r3, [1])
	#r3.append(1)
	self.perspectiveTf = [r1, r2, r3]
	return self.perspectiveTf
	
    # Convert points from worldCoordinates to image coordinates
    # worldCoordinates [3-tuples]
    # Return: [2-tuples]
    def computeImageCoordinates(self, worldCoordinates):
	iCoords = []
	i = 0
	for coord in worldCoordinates:
	    coord.append(1)
	    wc = self.ptToCamera(coord)
	    #wc = np.array(coord)
	    #wc = wc[np.newaxis, :].T
	    imgPt = np.dot(self.camMatrix, wc)
	    imgPt = np.around(imgPt,0) # Quantize
	    iCoords.append(imgPt)
	    i = i + 1
	return iCoords
	# Compute image coordinate with quantization
	
    # Image to world estimate coordinate transform with quantization (no added noise)
    def computeImageToWorldEstimate(self, imgCoordinates):
	weCoords = []
	i = 0
	for coord in imgCoordinates:
	    #c = np.append(coord, [1])
	    c = coord[np.newaxis, :].T
	    estPt = np.dot(self.perspectiveTf, c)
	    estPt = np.around(estPt, 3) # Estimated quantization from remapping
	    estPt = np.append(estPt,[[1]])
	    estPt.reshape((4,1))
	    estPt = self.ptToWorld(estPt)# Handle camera translation/rotation
	    #print 'Est Pt:',str(estPt)
	    weCoords.append([estPt[0],estPt[2]])
	    i = i + 1
	return weCoords
    
    # Converts a world point into an estimated world point (w/ noise)
    def computeRemappedCoordinate(self, worldCoordinates, noiseSigma):
	#print str(worldCoordinates)
	imgCoords = self.computeImageCoordinates(worldCoordinates) # Compute and quantize image coordinates
	weCoords = self.computeImageToWorldEstimate(imgCoords) # Reproject using rectification transform and quantize
	for coord in weCoords: # Additive Gaussian noise
	    coord[0] = coord[0] + np.random.normal(0, noiseSigma, 1)
	    coord[1] = coord[1] + np.random.normal(0, noiseSigma, 1)
	return weCoords
    
    # From 3 points, estimate center, theta (base rotation), phi (absolute laser rotation)
    def estimateState(self, points, idealFlag):
	gACtr = points[0]
	rACtr = points[1]
	bACtr = points[2]
	remapCtrs = self.computeRemappedCoordinate([gACtr, rACtr, bACtr],0.5)
	gCtr = remapCtrs[0]
	rCtr = remapCtrs[1]
	bCtr = remapCtrs[2]
	gx = float(gCtr[0])
	gy = float(gCtr[1])
	rx = float(rCtr[0])
	ry = float(rCtr[1])
	bx = float(bCtr[0])
	by = float(bCtr[1])
	# TODO: Run points through vision model
	# Compute mean of points 1 and 2
	ctr = np.array([[(gx + rx)/2],[(gy + ry)/2]]) # Compute line midpoint
	theta = atan2(gy-ry, gx-rx) # Compute base angle
	phi = atan2(by-ctr[1], bx-ctr[0]) # Compute laser angle
	if idealFlag:
	    ctr = np.array([[(gACtr[0] + rACtr[0])/2],[(gACtr[2] + rACtr[2])/2]]) # Compute line midpoint
	    theta = atan2(gACtr[2]-rACtr[2], gACtr[0]-rACtr[0]) # Compute base angle
	    phi = atan2(bACtr[2]-ctr[1], bACtr[0]-ctr[0]) # Compute laser angle
	    
	return ctr, theta, phi
	

    
    