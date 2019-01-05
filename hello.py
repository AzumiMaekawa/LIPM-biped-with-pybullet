# pybullet first test file

import pybullet as p
import time
import pybullet_data

pysicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)

# camera view setting
# args: cameraEyePosition, cameraTargetPosition, cameraUpVector
#viewMatrix = p.computeViewMatrix([0.5,-0.5,0.5],[0,0,0],[-1,1,2])
#p.getCameraImage(600,600,viewMatrix)

p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

distance = 2
yaw = 20
height = -30
p.resetDebugVisualizerCamera(distance, yaw, height, [0,0,0])

planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("/urdf/lip_model01.urdf",cubeStartPos, cubeStartOrientation)
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()
