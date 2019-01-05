# pybullet first test file

import pybullet as p
import time
import pybullet_data
import cv2 as cv
import numpy as np

pysicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)

# camera view setting
# args: cameraEyePosition, cameraTargetPosition, cameraUpVector
#viewMatrix = p.computeViewMatrix([0.5,-0.5,0.5],[0,0,0],[-1,1,2])
#p.getCameraImage(600,600,viewMatrix)

p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

distance = 1
yaw = 45 
pitch = -30
p.resetDebugVisualizerCamera(distance,yaw,pitch, [0,0,0])

planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0.7]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("/urdf/lip_model01.urdf",cubeStartPos, cubeStartOrientation)
#p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4,'test.mp4')

image_path = "images/" # define a path for image restore
p.setRealTimeSimulation(False)

cameraParams = p.getDebugVisualizerCamera() # to get the width and height of rendering frame
#print('width: {0}, height:{1}'.format(cameraParams[0],cameraParams[1]))
image_width  = cameraParams[0]
image_height = cameraParams[1]

for i in range(10000):
    p.stepSimulation()
    img_arr=p.getCameraImage(image_width,image_height, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    img = np.reshape(img_arr[2],(image_height,image_width,4)) 
    img = cv.cvtColor(img, cv.COLOR_RGBA2BGRA)# convert RGBA image to BGRA image (openCV:BGRA)
#    img = img*(1./255.)
    cv.imwrite(image_path+"{}.png".format(i), img)
    time.sleep(1./240.)
    if i == 1000:
        break
    

#p.stopStateLogging(STATE_LOGGING_VIDEO_MP4, test)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()
