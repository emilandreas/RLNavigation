import sys
import time
import matplotlib.pyplot as plt
import numpy as np
import pandas
from PIL import Image

sys.path.append('./vrep_lib')
from vrep_lib import vrep


#Init connection to V-REP
vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Failed connecting to remote API server. Exiting...')
    sys.exit(-1)


# Get handles for objects in V-REP
errorCode, left_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
if errorCode != -1:
    print("left:Error code: {}".format(errorCode))

errorCode, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
if errorCode != -1:
    print("right:Error code: {}".format(errorCode))

errorCode = vrep.simxGetObjectHandle(clientID, 'Path', vrep.simx_opmode_oneshot_wait)
if errorCode != -1:
    print("path:Error code: {}".format(errorCode))

errorCode, cam_handle = vrep.simxGetObjectHandle(clientID, 'Cam1', vrep.simx_opmode_oneshot_wait)
if errorCode != -1:
    print("cam:Error code: {}".format(errorCode))


# Set wheelspeed of robot
errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, 10, vrep.simx_opmode_oneshot_wait)
if errorCode != -1:
    print("setVel:Error code: {}".format(errorCode))



#################################
# # Get image from testCam in V-REP
# errorCode, resolution, image = vrep.simxGetVisionSensorImage(clientID, cam_handle, 0, vrep.simx_opmode_streaming)
# if errorCode != -1:
#     print("getImage:Error code: {}".format(errorCode))
# while(errorCode != 0):
#     errorCode, resolution, image = vrep.simxGetVisionSensorImage(clientID,
#                                                                  cam_handle, 0, vrep.simx_opmode_buffer)
#     time.sleep(1)
#
#
#
#
# plt.ion()
# im = Image.new("RGB", (resolution[0], resolution[1]), "white")
#
# fig = plt.figure(1)
# fig.canvas.set_window_title("livestream")
# #inverse the picture
# plotimg = plt.imshow(im,origin='lower')
# #Let some time to Vrep in order to let him send the first image, otherwise the loop will start with an empty image and will crash
#
#
# # Get continuous stream of images from V-REP
# while (vrep.simxGetConnectionId(clientID)!=-1):
#     errorCode, resolution, image = vrep.simxGetVisionSensorImage(clientID,
#                                                                  cam_handle, 0, vrep.simx_opmode_buffer)
#     #Transform the image so it can be displayed using pyplot
#     img = np.array(image, dtype=np.uint8)
#     img.resize([resolution[0], resolution[1], 3])
#     # image_byte_array = array.array('b',image)
#     # im = Image.frombuffer("RGB", (resolution[0], resolution[1]), image_byte_array, "raw", "RGB", 0, 1)
#     #Update the image
#     plotimg.set_data(img)
#     #Refresh the display
#     plt.draw()
#     #The mandatory pause ! (or it'll not work)
#     plt.pause(0.0001)
#
##########################


path = pandas.read_csv('./path1.csv',',')
path_array = path.as_matrix()
path_array = path_array[:,:6]
