#!/usr/bin/env python3

import cv2
import sys
import math
import time
import numpy as np
import traceback
import jetson.inference
import jetson.utils

from datetime import datetime
from utils.mtcnn import TrtMtcnn
from threading import Thread
from djitellopy import Tello

currentFrame = False
keepRecording = True
video_fps = 15
low_bat = 15
winname = "Tello"

# max velocity settings
max_yaw_velocity = 50
max_up_down_velocity = 40
max_forward_backward_velocity = 40

net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)

tello = Tello()
tello.connect()

battery = tello.get_battery()
print ("Battery: ", tello.get_battery());

if battery < low_bat:
    print ("Battery low")
    exit()

def limitVelocity(velocity, max_velocity):
    return min(max_velocity, max(-max_velocity, velocity))


def videoRecorder():
    global currentFrame
    now = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    height, width, _ = frame_read.frame.shape
    video = cv2.VideoWriter('../video/video-' + now + '.avi', cv2.VideoWriter_fourcc(*'XVID'), video_fps, (width, height))

    while keepRecording:
        start = time.time()
        currentFrame = frame_read.frame
        video.write(currentFrame)
        end = time.time();
        elapsed = end - start
        time.sleep(max(0,1 / video_fps - elapsed))
        end = time.time();
        # print("Record Video: {:0>1.0f} FPS".format(1 / (end - start)));

    video.release()

def dist(p1,p2):
    return math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )

def detectionDistance(detectionElem):
    if detection == False:
        return 0
    return dist(detectionElem.Center,detection.Center)

def getDetectedPerson(detections, lastDetection):
    persons = []
    for detection in detections:
        className = net.GetClassDesc(detection.ClassID)
        if className == 'person':
            persons.append(detection)
    persons.sort(key=detectionDistance)
    if len(persons) > 0:
        return persons[0]
    return False

tello.streamon()
frame_read = tello.get_frame_read()

recorder = Thread(target=videoRecorder)
recorder.start()

tello.send_rc_control(0, 0, 0, 0)
tello.takeoff()
tello.move_up(50)

try:
    running = True
    detection = False
    while running:
        start = time.time()
        battery = tello.get_battery();
        if battery < low_bat:
            print ("Battery low")
            running = False

        cuda_img = jetson.utils.cudaFromNumpy(currentFrame, isBGR=False)
        detections = net.Detect(cuda_img, overlay='none')
        np_img = jetson.utils.cudaToNumpy(cuda_img)
        h,w,c = np_img.shape
        yaw_velocity = 0
        left_right_velocity = 0
        up_down_velocity = 0
        forward_backward_velocity = 0
        if len(detections) > 0:
            detection = getDetectedPerson(detections, detection)
            if detection != False:
                color = (0, 255, 0)
                np_img = cv2.rectangle(np_img, (int(detection.Left), int(detection.Top)), (int(detection.Right), int(detection.Bottom)), color, 2)
                np_img = cv2.circle(np_img, (int(detection.Center[0]),int(detection.Center[1])), 5, color, 5, cv2.LINE_AA)
                detectX = int(detection.Center[0])
                centerX = w / 2;
                yaw_velocity = int((detectX - centerX) / 7) # proportional only 
                up_down_velocity = int ((30 - detection.Top) / 2)
                forward_backward_velocity = int((h - detection.Bottom - 30) / 2) 

                # yaw_velocity = min(40, max(-40, yaw_velocity)) 
                # up_down_velocity = min(20, max(-20, up_down_velocity)) 
                # forward_backward_velocity = min(20, max(-20, forward_backward_velocity)) 
                yaw_velocity = limitVelocity(yaw_velocity, max_yaw_velocity)
                up_down_velocity = limitVelocity(up_down_velocity, max_up_down_velocity) 
                forward_backward_velocity = limitVelocity(forward_backward_velocity, max_forward_backward_velocity)
                left_right_velocity = 0
                # TODO left_right_velocity = 20

                print (w,h,detection.Top, detection.Bottom, forward_backward_velocity, up_down_velocity)
                print (centerX, yaw_velocity)

            else:
                print ("No Person detected")
        else:
            print ("No detection")

        tello.send_rc_control(left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)

        # Measure Time and show window
        end = time.time()

        # scale image
        scale_percent = 50 # percent of original size
        width = int(np_img.shape[1] * scale_percent / 100)
        height = int(np_img.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv2.resize(np_img, dim, interpolation = cv2.INTER_AREA)

        cv2.namedWindow(winname)    
        cv2.moveWindow(winname, 0,0)
        cv2.imshow(winname,resized)

        keyCode = cv2.waitKey(1) & 0xFF
        if keyCode == 27:
            break

except: # catch *all* exceptions
    print ("Exception")
    # TODO remove
    var = traceback.format_exc()
    print (var)
    e = sys.exc_info()[0]
    print("Error", e)

cv2.destroyAllWindows()

print("Land")

keepRecording = False
recorder.join()

tello.send_rc_control(0, 0, 0, 0)
battery = tello.get_battery()
print ("Battery: ", tello.get_battery());
tello.land()


