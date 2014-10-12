def printCentroids(gCtr, rCtr):
	if((gCtr != None) and (rCtr != None)):
		gx = round(gCtr[0],2)
		gy = round(gCtr[1],2)
		rx = round(rCtr[0],2)
		ry = round(rCtr[1],2)
		print 'Green:(', str(gx), ',', str(gy), ') ; Red:(', str(rx), ',', str(ry), ')'

def comboImage(gImg, rImg):
	zeroArr = np.zeros(gImg.shape, dtype=np.uint8)
	return cv2.merge((zeroArr,gImg, rImg))