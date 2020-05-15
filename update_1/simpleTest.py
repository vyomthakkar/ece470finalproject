#!/usr/bin/env python
# encoding: utf-8
 
"""
Enable the vision sensor in V-REP,Python
use the scene：VisionSensorDemo.ttt
 
@Author: Zane
@Contact: ely.hzb@gmail.com
@File: VisionSensorDemo.py
@Time: 2019-07-23 15:55
"""
import sim
import sys
import numpy as np
import math
import matplotlib.pyplot as mpl
import time
 
class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg
 
def main(argv=None):
    if argv is None:
        argv = sys.argv
   
    #Python connect to the V-REP client
    sim.simxFinish(-1)
   
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
   
    if clientID != -1:
        print("Connected to remote API server")
    else:
        print("Connection not successful")
        sys.exit("Connected failed,program ended!")
   
    #Get the handle of vision sensor
    errorCode,visionSensorHandle = sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_oneshot_wait)
   
    #Get the image of vision sensor
    errprCode,resolution,image = sim.simxGetVisionSensorImage(clientID,visionSensorHandle,0,sim.simx_opmode_streaming)
    time.sleep(0.1)
    errprCode,resolution,image = sim.simxGetVisionSensorImage(clientID,visionSensorHandle,0,sim.simx_opmode_buffer)
   
    #Process the image to the format (64,64,3)
    sensorImage = []
    sensorImage = np.array(image,dtype = np.uint8)
    sensorImage.resize([resolution[0],resolution[1],3])
   
    #Use matplotlib.imshow to show the image
    mpl.imshow(sensorImage,origin='lower')
 
 
if __name__ == "__main__":
    sys.exit(main())