#!/bin/python3
from asyncio import wait_for
import socket
import time
from typing import List
import cv2 as cv
from visionops import process
import qwiic_vl53l1x

# TODO 
# - Serial communication to Arduino 
# - Arduino code (receive command, drive stepper)
# - Generate map function from list of contours
# - Client program - Spacebar to command start, socket to send commands / receive map

ROBOT_HOST = '192.168.125.1'
ROBOT_PORT = 5024
robot_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

HMI_HOST = ''
HMI_PORT = 1123
hmi_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

while True:
    try:
        robot_client.connect((ROBOT_HOST, ROBOT_PORT))
        break
    except:
        print("Failed to connect to Robot, retrying in 1 second...")

    time.sleep(1)
print("Robot Connected.")

def send_to_robot(cmd):
    print("Sent \"" + cmd + "\" command to Jetson")
    robot_client.send(cmd.encode())
def recv_from_robot():
    retval = ""
    try:
        retval = robot_client.recv(1024).decode()
    except:
        pass

    return retval

while True:
    try:
        hmi_client.connect((HMI_HOST, HMI_PORT))
        break
    except:
        print("Failed to connect to HMI, retrying in 1 second...")

    time.sleep(1)

print("HMI Connected.")

# 1 = error
# 0 = OK
def wait_until(condition, timeout, interval):
    start = time.time()
    while not condition():
        end = time.time()
        if(end - start > timeout):
            return 1
    0
        
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
        
# Initialize Camera (jetson CSI port)
vc = cv.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink")

# Initialize LIDAR (i2c 1, address 0x52)
lidar = qwiic_vl53l1x.QwiicVL53L1X()
retval = lidar.sensor_init()
if retval == None:
    print("Lidar initialized")
else:
    print("Failed to initialize Lidar")
lidar.set_distance_mode(1)

while True:
    # Step 0 - Wait for HMI command to start!
    # Step 1 - take initial picture
    print("Taking picture...")
    send_to_robot("PIC") # Tell jetson to go to "picture position"
    wait_for_ok()
    wait_for_done()

    defect_list, vis_map = process(vc) # Take the picture & process for possible defects

    dist_list = []
    # Step 2 - Scan each point of interest for lidar depth
    for poi in defect_list:
        send_to_robot("LGOTO " + poi[0] + " " + poi[1])
        wait_for_ok()
        wait_for_done()
        # Get lidar value
        d = get_lidar_mm()
        dist_list.append(d) #TODO offset needed
        print("X: " + poi[0] + "mm Y: " + poi[1] + "mm A: " + poi[2] + "mm^2 D: " + d + "mm")

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
