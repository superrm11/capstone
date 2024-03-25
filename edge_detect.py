import cv2 as cv
import numpy as np
import sys

min_area = 300
max_area = 10000
def filter(contours):
    out = []
    for i in range(len(contours)):
        a = cv.contourArea(contours[i])
        if(a > min_area and a < max_area):
            out.append(contours[i])
    return out

canny_thresh = 12
canny_ratio = 3 # per OpenCV recommendation
dilation = 10
erosion = 10
blur_kernel = 6

def edge_detect(img):

    # Avoid the program crashing because of a large amount of contours from canny
    if(canny_thresh <= 1):
        return np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

    # Step 1 - blur the image to remove small imperfections from possible defects
    grayscaled = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    blured = cv.blur(grayscaled, (blur_kernel, blur_kernel))

    # Step 2 - perform canny edge detection, find contours & store them
    edge = cv.Canny(blured, canny_thresh, canny_thresh*canny_ratio)
    contours,_  = cv.findContours(edge, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # Step 3 - Convex hull operation to fill in holes (in the hole), convert to binary image
    hull_drawing = np.zeros((edge.shape[0], edge.shape[1], 1), dtype=np.uint8)
    hull_list = []
    for i in range(len(contours)):
        hull = cv.convexHull(contours[i])
        hull_list.append(hull)
        cv.drawContours(hull_drawing, [hull], 0, 255, cv.FILLED)
    
    # Step 4 - Dilate the blob to further fill in holes
    if(dilation > 0): 
        element = cv.getStructuringElement(cv.MORPH_RECT, (dilation, dilation))
        hull_drawing = cv.dilate(hull_drawing, element)

    # Step 5 - Erode the blob to restore the size
    if(erosion > 0):
        element = cv.getStructuringElement(cv.MORPH_RECT, (erosion, erosion))
        hull_drawing = cv.erode(hull_drawing, element)


    # Find the final contours, draw a bounding box on the original image
    final_contours,_ =  cv.findContours(hull_drawing, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    final_contours = filter(final_contours)
    final_drawing = cv.copyTo(img, None)
    for i in range(len(final_contours)):
        x,y,w,h = cv.boundingRect(final_contours[i])
        cv.rectangle(final_drawing, (x,y), (x+w, y+h), (0,255,0), thickness=5)

    # Create a debug window    
    h = int(img.shape[0] / 2)
    w = int(img.shape[1] / 2)
    # Top left: original
    tl = cv.resize(img, (w, h))
    # Top right: blur
    tr = cv.cvtColor(blured, cv.COLOR_GRAY2RGB)
    tr = cv.resize(tr, (w, h))
    # tr = cv.cvtColor(blured, cv.COLOR_GRAY2RGB)
    # Bottom left: Convex hull
    bl = cv.cvtColor(hull_drawing, cv.COLOR_GRAY2RGB)
    bl = cv.resize(bl, (w, h))
    # Bottom right: original w/ box
    br = cv.resize(final_drawing, (w,h))

    top = np.hstack((tl, tr))
    bottom = np.hstack((bl, br))
    debug_frame = np.vstack((top, bottom))

    # cv.imshow("blur", blured)

    return debug_frame

image = cv.imread("drywall.jpg")
image = cv.resize(image, (0,0), fx=0.5, fy=0.5)

cv.namedWindow("Contours")

def cannyThreshTrackbar(newValue):
    global canny_thresh
    canny_thresh = newValue
    cv.imshow("Contours", edge_detect(image))

def kernelSizeTrackbar(newValue):
    global blur_kernel
    blur_kernel = newValue
    cv.imshow("Contours", edge_detect(image))

def erosionTrackbar(newValue):
    global erosion
    erosion = newValue
    cv.imshow("Contours", edge_detect(image))

def dilationTrackbar(newValue):
    global dilation
    dilation = newValue
    cv.imshow("Contours", edge_detect(image))

def minareaTrackbar(newValue):
    global min_area
    min_area = newValue
    cv.imshow("Contours", edge_detect(image))

def maxareaTrackbar(newValue):
    global max_area
    max_area = newValue
    cv.imshow("Contours", edge_detect(image))

cv.createTrackbar("canny thresh", "Contours", canny_thresh, 50, cannyThreshTrackbar)
cv.createTrackbar("blur kernel", "Contours", blur_kernel, 100, kernelSizeTrackbar)
cv.createTrackbar("dilation", "Contours", dilation, 50, dilationTrackbar)
cv.createTrackbar("erosion", "Contours", erosion, 50, erosionTrackbar)
cv.createTrackbar("min area", "Contours", min_area, 10000, minareaTrackbar)
cv.createTrackbar("max area", "Contours", max_area, 10000, maxareaTrackbar)

cv.imshow("Contours", edge_detect(image))
while(cv.waitKey(0) != ord('q')):
    pass