import cv2 as cv
import numpy as np
import sys

lower_thresh = 0
upper_thresh = 255
blur_kernel = 3

def threshold(img):
    grayscaled = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    blured = cv.blur(grayscaled, (blur_kernel, blur_kernel))

    thresh = cv.inRange(blured, lower_thresh, upper_thresh)

    contours,_  = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    hull_list = []
    for i in range(len(contours)):
        hull = cv.convexHull(contours[i])
        hull_list.append(hull)
        

    contour_drawing = np.zeros((thresh.shape[0], thresh.shape[1], 3), dtype=np.uint8)
    for i in range(len(hull_list)):
        rot_rect = cv.minAreaRect(hull_list[i])
        box = cv.boxPoints(rot_rect)
        # cv.drawContours(contour_drawing, contours, i, (0, 0, 255))
        cv.drawContours(contour_drawing, hull_list, i, (0, 0, 255), cv.FILLED)
        # cv.drawContours(contour_drawing, [box], 0, (0, 255, 0))
        
    cv.imshow("blur", blured)

    return contour_drawing

image = cv.imread("/shared/ryan/Downloads/drywall.jpg")
image = cv.resize(image, (0,0), fx=0.5, fy=0.5)


cv.namedWindow("Contours")

def lThreshTrackbar(newValue):
    global lower_thresh
    lower_thresh = newValue
    cv.imshow("Contours", threshold(image))

def uThreshTrackbar(newValue):
    global upper_thresh
    upper_thresh = newValue
    cv.imshow("Contours", threshold(image))

def kernelSizeTrackbar(newValue):
    global blur_kernel
    blur_kernel = newValue
    cv.imshow("Contours", threshold(image))


cv.createTrackbar("lower thresh", "Contours", 0, 255, lThreshTrackbar)
cv.createTrackbar("upper thresh", "Contours", 0, 255, uThreshTrackbar)
cv.createTrackbar("blur kernel", "Contours", 1, 20, kernelSizeTrackbar)

# cv.imshow("Grayscale", image)
cv.imshow("Contours", threshold(image))
cv.waitKey(0)