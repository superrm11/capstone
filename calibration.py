import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# vc = cv.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink drop=1")
# index = 0
# while(not cv.waitKey(5) == ord('q')):
#     ret, src = vc.read()
#     img = cv.cvtColor(src, cv.COLOR_RGB2GRAY)
    
#     print("finding contours...")
#     ret, corners = cv.findChessboardCorners(img, (9, 6), None)
#     print("eh?")
#     if ret:
#         print("Found!")
#         prev = cv.drawChessboardCorners(src, (9,6), corners, ret)
#         cv.imshow("chess", prev)
#         if(cv.waitKey(0) == ord('y')):
#             cv.imwrite("/home/jetson/calibration/pic"+index+".jpg", src)
#             index+=1
#     else:
#         print("None found... :(")

# exit()

images = glob.glob('robot_calibration/*.jpg')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
    else:
        print("Unable to find corners")
        continue
    
    corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
    imgpoints.append(corners2)

    # Draw and display the corners
    cv.drawChessboardCorners(img, (9,6), corners2, ret)
    cv.imshow('img', img)
    cv.waitKey(500)

cv.destroyAllWindows()
print(f'obj:{len(objpoints)}, img:{len(imgpoints)}')
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print(f'mtx:\n{mtx}\ndist:\n{dist}\nrvecs:\n{rvecs}\ntvecs:\n{tvecs}')