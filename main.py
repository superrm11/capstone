#!/bin/python3
from asyncio import wait_for
import socket
import time
from typing import List
import cv2 as cv
from visionops import process
import qwiic_vl53l1x
import serial

# LIVE VIEW TESTING
# vc = cv.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1640, height=(int)1232,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink drop=1")
# while True:
#     try:
#         vals, drawing = process(vc)
#     except:
#         vc.release()
#     for pt in vals:
#         cv.putText(drawing, 'X:{} Y:{}'.format(round(pt[0]), round(pt[1])), (int(round(pt[3])), int(round(pt[4])-20)), cv.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
#     drawing = cv.resize(drawing, (1230, 924))
#     cv.imshow("Map", drawing)
#     # print(vals)
#     key = cv.waitKey(1)
#     if (key == ord('y')):
#         cv.imwrite('output.jpg', drawing)
#     if (key == ord('q')):
#         break
# vc.release()
# exit(0)

# Camera Calibration Code
# vc = cv.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink drop=1")
# index = 10
# while True:
#     ret, img = vc.read()
#     cv.imshow("calib", img)
#     key = cv.waitKey()
#     if (key == ord('y')):
#         cv.imwrite('/home/jetson/calibration/pic{}.jpg'.format(index), img)
#         index+=1
#     elif (key == ord('q')):
#         break
# vc.release()
# exit(0)

# LIDAR Testing Code
# def get_lidar_mm():
#     lidar.start_ranging()
#     time.sleep(.005)
#     dist = lidar.get_distance()
#     time.sleep(.005)
#     lidar.stop_ranging()
#     return dist
# lidar = qwiic_vl53l1x.QwiicVL53L1X()
# retval = lidar.sensor_init()
# if retval == None:
#     print("Lidar initialized")
# else:
#     print("Failed to initialize Lidar")
# lidar.set_distance_mode(1)
# while True:
#     print(get_lidar_mm())
#     time.sleep(0.1)

# TODO 
# - Mask out-of-bounds area before canny
# - Serial communication to Arduino 
# - Arduino code (receive command, drive stepper)
# - Generate map function from list of contours
# - Client program - Spacebar to command start, socket to send commands / receive map

# Initialize Camera (jetson CSI port)
try:
    vc = cv.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1640, height=(int)1232,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink drop=1")
except:
    print("Error starting camera - aborting...")
    exit(1)
# Initialize LIDAR (i2c 1, address 0x52)
lidar = qwiic_vl53l1x.QwiicVL53L1X()
retval = lidar.sensor_init()
if retval == None:
    print("Lidar initialized")
else:
    print("Failed to initialize Lidar")
lidar.set_distance_mode(1)

port = serial.Serial("/dev/ttyACM0", 9600)
print("Serial initialized")


ROBOT_HOST = '10.3.39.4'
ROBOT_PORT = 5024
robot_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Wait for a connection to the robot
while True:
    try:
        robot_client.connect((ROBOT_HOST, ROBOT_PORT))
        break
    except:
        print("Failed to connect to Robot, retrying in 1 second...")

    time.sleep(1)
print("Robot Connected.")

def send_to_robot(cmd):
    print("Sent \"" + cmd + "\" command to Robot")
    robot_client.send(cmd.encode())

def recv_from_robot():
    retval = ""
    try:
        retval = robot_client.recv(1024).decode()
    except:
        pass

    return retval

# 1 = error
# 0 = OK
def wait_until(condition, timeout, interval):
    start = time.time()
    while not condition():
        end = time.time()
        if(end - start > timeout):
            return 1
        
def wait_for_ok():
    print("Waiting for OK signal...")
    ret = wait_until(lambda: recv_from_robot() == "OK", 5, 0.1)
    if(ret == 1):
        print("Communication Error!")
        exit(1)
    print("OK received. Continuing...")

def wait_for_done():
    print("Waiting for DONE signal...")
    ret = wait_until(lambda: recv_from_robot() == "DONE", 10, 0.1)
    if(ret == 1):
        print("Watchdog error - robot took too long!")
        exit(1)
    print("DONE received. Continuing...")

def get_lidar_mm():
    lidar.start_ranging()
    time.sleep(.005)
    print(lidar.get_distance())
    time.sleep(.005)
    lidar.stop_ranging()

cv.namedWindow("Defect Map")
cv.startWindowThread()
def show_map(img, pts):
    for pt in pts:
        cv.putText(img, 'X:{} Y:{}'.format(round(pt[0]), round(pt[1])), (int(round(pt[3])), int(round(pt[4])-20)), cv.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
    img = cv.resize(img, (1230, 924))
    cv.imshow("Defect Map", img)

while True:
    # input("Press Enter to continue.")

    # Step 0 - Wait for HMI command to start!
    # Step 1 - take initial picture
    print("Taking picture...")
    send_to_robot("PIC") # Tell jetson to go to "picture position"
    wait_for_ok()
    wait_for_done()

    start_time = time.time()
    vis_map = None
    defect_list = None
    while (time.time() - start_time < 2):
        # Allow white balence & exposure to settle
        defect_list, vis_map = process(vc)
        show_map(vis_map, defect_list)

    defect_list, vis_map = process(vc)
    show_map(vis_map, defect_list)

    send_to_robot("ENDPIC")
    wait_for_ok()
    wait_for_done()

    print("{} Defects Found!".format(len(defect_list)))

    for pt in defect_list:
        print("Dispensing Spackle...")
        send_to_robot("DISP")
        wait_for_ok()
        wait_for_done()

        # port.write("F".encode())
        port.write("F".encode())
        port.write("B".encode())
        # port.write("B".encode())
        time.sleep(3)

        print("Repairing at {}, {}".format(pt[0], pt[1]))
        send_to_robot("RGOTO {},{}".format(int(pt[0]), int(pt[1])))
        wait_for_ok()
        wait_for_done()

    input("Press Enter to End.")
    cv.destroyAllWindows()
    exit(0)

    # dist_list = []
    # Step 2 - Scan each point of interest for lidar depth
    # for poi in defect_list:
    #     send_to_robot("LGOTO " + poi[0] + " " + poi[1])
    #     wait_for_ok()
    #     wait_for_done()
    #     # Get lidar value
    #     d = get_lidar_mm()
    #     dist_list.append(d) #TODO offset needed
    #     print("X: " + poi[0] + "mm Y: " + poi[1] + "mm A: " + poi[2] + "mm^2 D: " + d + "mm")

    # Cut out points that don't meet the depth / area requirement
    final_poi_list = []
    offset = 0 # Tune me based on distance!
    for i in range(len(defect_list)):
        a = defect_list[i][2]
        d = dist_list[i]

        if(a > 0 and d > 0): #Tune min area and distance!
            final_poi_list.append([defect_list[i][0], defect_list[i][1], defect_list[i][2], dist_list[i]])
        


while(cv.waitKey(5) != ord('q')):
    arr = process(vc)
    if (len(arr) > 0):
        print(arr)
