# pybullet first test file
# render frame check (main camera position, render frame video)

import pybullet as p
import time
import pybullet_data
import numpy as np

pysicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.8)

# hide sub-camera
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

# camera positioin setting
distance = 1
yaw = 45
pitch = -30
p.resetDebugVisualizerCamera(distance, yaw, pitch, [0, 0, 0])

# plane setting
planeId = p.loadURDF("plane.urdf")

# intial setting of the agent
startPos = [0, 0, 0.7]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
walkerId = p.loadURDF("/urdf/lip_model01.urdf", startPos, startOrientation)
# boxId = p.loadURDF("/urdf/leg.urdf",cubeStartPos, cubeStartOrientation)

mode = p.POSITION_CONTROL
maxForce = 5.0


p.setRealTimeSimulation(False)


if __name__ == "__main__":
    for i in range(5000):
        p.stepSimulation()
        time.sleep(1./240.)
        if i == 200:
            p.setJointMotorControl2(
                walkerId, 1, controlMode=mode, force=maxForce, targetPosition=1.0)

    pos, orn = p.getBasePositionAndOrientation(walkerId)
    print(pos, orn)
    p.disconnect()
