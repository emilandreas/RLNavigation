import sys
import time
import matplotlib.pyplot as plt
import numpy as np
import pandas
import Simulator as s
from PIL import Image
import xbox
sys.path.append('./vrep_lib')
from vrep_lib import vrep
import robot_control

class Simulator:
    clientID = -1
    handles = {}
    robo = robot_control.Pioneer_robo_control
    def __init__(self):
        #Init connection to V-REP
        vrep.simxFinish(-1)
        self.clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
        if self.clientID!=-1:
            print ('Connected to remote API server')
        else:
            print ('Failed connecting to remote API server. Exiting...')
            sys.exit(-1)

        # Init handles
        errorCode, left_motor_handle = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
        if errorCode != -1:
            print("left:Error code: {}".format(errorCode))

        errorCode, robot_handle = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx', vrep.simx_opmode_oneshot_wait)
        if errorCode != -1:
            print("robot:Error code: {}".format(errorCode))

        errorCode, right_motor_handle = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
        if errorCode != -1:
            print("right:Error code: {}".format(errorCode))

        errorCode, cam_handle = vrep.simxGetObjectHandle(self.clientID, 'Cam1', vrep.simx_opmode_oneshot_wait)
        if errorCode != -1:
            print("cam:Error code: {}".format(errorCode))

        errorCode, cam_handle = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
        if errorCode != -1:
            print("cam:Error code: {}".format(errorCode))

        self.handles.update({"left_motor_handle": left_motor_handle,
                        "right_motor_handle": right_motor_handle,
                        "robot_handle": robot_handle,
                        "cam_handle":cam_handle
                        })

        self.robo = robot_control.Pioneer_robo_control(self.clientID,self.handles["robot_handle"],
                                                  self.handles["right_motor_handle"], self.handles["left_motor_handle"])


    def init_video_stream(self):
        errorCode, resolution, image = vrep.simxGetVisionSensorImage(self.clientID, self.handles["cam_handle"], 0, vrep.simx_opmode_streaming)
        if errorCode != -1:
            print("getImage:Error code: {}".format(errorCode))
        while(errorCode != 0):
            errorCode, resolution, image = vrep.simxGetVisionSensorImage(self.clientID,
                                                                         self.handles["cam_handle"], 0, vrep.simx_opmode_buffer)
            time.sleep(1)
        plt.ion()
        im = Image.new("RGB", (resolution[0], resolution[1]), "white")

        self.fig = plt.figure(1)
        self.fig.canvas.set_window_title("livestream")
        #inverse the picture
        self.plotimg = plt.imshow(im,origin='lower')
    def show_video_stream(self):

        errorCode, resolution, image = vrep.simxGetVisionSensorImage(self.clientID,
                                                                     self.handles["cam_handle"], 0, vrep.simx_opmode_buffer)
        #Transform the image so it can be displayed using pyplot
        img = np.array(image, dtype=np.uint8)
        img.resize([resolution[0], resolution[1], 3])
        # image_byte_array = array.array('b',image)
        # im = Image.frombuffer("RGB", (resolution[0], resolution[1]), image_byte_array, "raw", "RGB", 0, 1)
        #Update the image
        self.plotimg.set_data(img)
        #Refresh the display
        plt.draw()
        #The mandatory pause ! (or it'll not work)
        plt.pause(0.0001)







def xbox_controller_steering(sim)
    joy = xbox.Joystick()
    while(True):
        vel = joy.leftY()
        steering = joy.leftX()
        sim.robo.control
