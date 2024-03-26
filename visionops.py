#!/bin/python3
import cv2 as cv
import numpy as np



def process(cam:cv.VideoCapture) -> list[tuple[float, float, float]]:
    """Takes a picture, undistorts and runs operations to find 
    information on defects

    Args:
        cam (cv.VideoCapture): Video capture device (already open)

    Returns:
        list[tuple[float, float, float]]: List of defect datapoints, in mm/mm^2 (x, y, area)
    """
    mtx=np.array([[1.43053474e+03, 0.00000000e+00, 7.37142021e+02],
        [0.00000000e+00, 1.40876133e+03, 5.39785147e+02],
        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    dist=np.array([[-0.24913637,  0.55272668,  0.00304293, -0.01298301, -0.51251071]])

    canny_thresh = 12
    canny_ratio = 3 # per OpenCV recommendation
    dilation = 10
    erosion = 10
    blur_kernel = 6

    ret, src = cam.read()
    if not ret:
        print("Unable to grab image!")
        return []

    # Correct for lens distortion (Must complete calibration first! mtx, dist!)
    src = cv.undistort(src, mtx, dist)

    # Avoid the program crashing because of a large amount of contours from canny
    if(canny_thresh <= 1 or blur_kernel < 1):
        return np.zeros((src.shape[0], src.shape[1], 3), dtype=np.uint8)

    # Step 1 - blur the image to remove small imperfections from possible defects
    grayscaled = cv.cvtColor(src, cv.COLOR_RGB2GRAY)
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
    final_drawing = cv.copyTo(src, None)
    for i in range(len(final_contours)):
        x,y,w,h = cv.boundingRect(final_contours[i])
        cv.rectangle(final_drawing, (x,y), (x+w, y+h), (0,255,0), thickness=5)

    # Create a debug window    
    h = int(src.shape[0] / 2)
    w = int(src.shape[1] / 2)
    # Top left: original
    tl = cv.resize(src, (w, h))
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

    # TODO Debug option to save pictures / videos to file

    # TODO map pixels to millimeters! (pinhole camera math)
    return [(0, 0, 0), (0, 0, 0)]