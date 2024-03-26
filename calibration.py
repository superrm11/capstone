import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

vc = cv.VideoCapture(0)

while(not cv.waitKey(5) == ord('q')):
    ret, src = vc.read()
    img = cv.cvtColor(src, cv.COLOR_RGB2GRAY)
    if(not ret):
        continue
    
    ret, corners = cv.findChessboardCorners(img, (9, 6), None)

    src = cv.drawChessboardCorners(src, (7,6), corners, ret)
    cv.imshow("chess", src)

exit()

images = glob.glob('*.jpg')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7,6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
    else:
        print("Unable to find corners")
        continue
    
    corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
    imgpoints.append(corners2)

    # Draw and display the corners
    cv.drawChessboardCorners(img, (7,6), corners2, ret)
    cv.imshow('img', img)
    cv.waitKey(500)

cv.destroyAllWindows()
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print(f'mtx:\n{mtx}\ndist:\n{dist}\nrvecs:\n{rvecs}\ntvecs:\n{tvecs}')
