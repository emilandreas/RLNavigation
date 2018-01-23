import sys
import time
import matplotlib.pyplot as plt
import numpy as np
import pandas
from Simulator import *
from PIL import Image
from math import pi
import helpers
sys.path.append('./vrep_lib')
from vrep_lib import vrep



#Init connection to V-REP
s = Simulator()

path = pandas.read_csv('./path1.csv',',')
path_array = path.as_matrix()
path_array = path_array[:,:6]

###################################################
# s.init_video_stream()
# while(vrep.simxGetConnectionId(s.clientID)!=-1):
#      s.show_video_stream()
###################################################

xbox_controller_steering(s)

s.robo.control_robo(1,1)
time.sleep(3)
s.robo.control_robo(0,0)
time.sleep(3)
s.robo.control_robo(-1,-1)
time.sleep(3)
s.robo.control_robo(1,0)
time.sleep(3)

vrep.simxPauseSimulation(s.clientID, vrep.simx_opmode_oneshot)
time.sleep(3)
vrep.simxStartSimulation(s.clientID, vrep.simx_opmode_oneshot)
time.sleep(3)
vrep.simxStopSimulation(s.clientID, vrep.simx_opmode_oneshot)
time.sleep(3)
vrep.simxStartSimulation(s.clientID, vrep.simx_opmode_oneshot)

errorCode, euler_angles = vrep.simxGetObjectOrientation(s.clientID, objectHandle=s.handles["robot_handle"], relativeToObjectHandle=-1, operationMode=vrep.simx_opmode_streaming)
errorCode, robo_pos = vrep.simxGetObjectPosition(s.clientID, objectHandle=s.handles["robot_handle"], relativeToObjectHandle=-1, operationMode=vrep.simx_opmode_streaming)
time.sleep(0.5)
errorCode = 0
while(errorCode==0):

    errorCode, euler_angles = vrep.simxGetObjectOrientation(s.clientID, objectHandle=s.handles["robot_handle"], relativeToObjectHandle=-1, operationMode=vrep.simx_opmode_buffer)
    errorCode, robo_pos = vrep.simxGetObjectPosition(s.clientID, objectHandle=s.handles["robot_handle"], relativeToObjectHandle=-1, operationMode=vrep.simx_opmode_buffer)
    angle_a = np.deg2rad*path_array[10, 5]
    angle_b = euler_angles[2]
    print("angles: {},{}, diff: {}, robo_pos: {}".format(angle_a, angle_b, helpers.get_angle_diff(angle_a, angle_b),0))
    # print(euler_angles)
    dist, i = helpers.get_dist_to_path(robo_pos, path_array[:,:3])
    print("dist_to_path: {}, closest_path_point: {}".format(dist, i))
    time.sleep(2)



path = pandas.read_csv('./path1.csv',',')
path_array = path.as_matrix()
path_array = path_array[:,:6]
