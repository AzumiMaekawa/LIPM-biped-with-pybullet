# pybullet first test file

import pybullet as p
import time
import pybullet_data
import cv2 as cv

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
pitch = -45
p.resetDebugVisualizerCamera(distance,yaw,pitch, [0,0,0])

planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0.5]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("/urdf/lip_model01.urdf",cubeStartPos, cubeStartOrientation)
#p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4,'test.mp4')

image_path = "images/"
p.setRealTimeSimulation(False)
for i in range(10000):
    p.stepSimulation()
    img=p.getCameraImage(500, 500, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    cv.imwrite(image_path+"{}.png".format(i), img)
    time.sleep(1./240.)
    if i == 500:
        break
    

#p.stopStateLogging(STATE_LOGGING_VIDEO_MP4, test)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()
