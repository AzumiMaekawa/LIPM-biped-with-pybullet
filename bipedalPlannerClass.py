import pybullet as p
import pybullet_data
import time
import math
import numpy as np

PI = math.pi


class BipedalPlanner(object):
    def __init__(self, urdfRootPath):
        self.urdfRootPath = urdfRootPath
        self.ref_footsteps = None
        self.reset()

    def reset(self):
        startPos = [0, 0, 0.7]
        startOri = p.getQuaternionFromEuler([0, 0, 0])
        self.ID = p.loadURDF(self.urdfRootPath, startPos, startOri)
        self.kp = 1
        self.kd = 0.1
        self.maxForce = 3.5
        self.nMotors = 6
        self.motorIdList = []
        self.motorDir = [-1, -1, -1, 1, 1, 1]
        self.buildJointNameToIdDict()
        self.buildMotorIdList()

    def buildJointNameToIdDict(self):
        nJoints = p.getNumJoints(self.ID)
        self.jointNameToId = {}
        for i in range(nJoints):
            # p.getJointInfo returns a list of information
            # [jointIndex, jointName, jointType, ...]
            jointInfo = p.getJointInfo(self.ID, i)
            # type of jointInfo[1] is <class 'bytes'>
            self.jointNameToId[jointInfo[1].decode('utf-8')] = jointInfo[0]
        self.resetPose()
        for i in range(100):
            p.stepSimulation()

    def buildMotorIdList(self):
        self.motorIdList.append(self.jointNameToId['leftHip_joint'])
        self.motorIdList.append(self.jointNameToId['leftKnee_joint'])
        self.motorIdList.append(self.jointNameToId['leftAnkle_joint'])
        self.motorIdList.append(self.jointNameToId['rightHip_joint'])
        self.motorIdList.append(self.jointNameToId['rightKnee_joint'])
        self.motorIdList.append(self.jointNameToId['rightAnkle_joint'])

    def setMotorAngleById(self, motorId, desiredAngle):
        p.setJointMotorControl2(bodyIndex=self.ID,
                                jointIndex=motorId,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=desiredAngle,
                                positionGain=self.kd,
                                force=self.maxForce)

    def setMotorAngleByName(self, motorName, desiredAngle):
        self.setMotorAngleById(self.jointNameToId[motorName], desiredAngle)

    def resetPose(self):

        hipAngle = - PI / 6
        kneeAngle = PI / 3
        ankleAngle = - PI / 6

        # left leg
        p.resetJointState(
            self.ID,
            self.jointNameToId['leftHip_joint'],
            self.motorDir[0]*hipAngle)
        p.resetJointState(
            self.ID,
            self.jointNameToId['leftKnee_joint'],
            self.motorDir[1]*kneeAngle)
        p.resetJointState(
            self.ID,
            self.jointNameToId['leftAnkle_joint'],
            self.motorDir[2]*ankleAngle)
        self.setMotorAngleByName('leftHip_joint', self.motorDir[0]*hipAngle)
        self.setMotorAngleByName('leftKnee_joint', self.motorDir[1]*kneeAngle)

        # right leg
        p.resetJointState(
            self.ID,
            self.jointNameToId['rightHip_joint'],
            self.motorDir[3]*hipAngle)
        p.resetJointState(
            self.ID,
            self.jointNameToId['rightKnee_joint'],
            self.motorDir[4]*kneeAngle)
        p.resetJointState(
            self.ID,
            self.jointNameToId['rightAnkle_joint'],
            self.motorDir[5]*ankleAngle)
        self.setMotorAngleByName('rightHip_joint', self.motorDir[3]*hipAngle)
        self.setMotorAngleByName('rightKnee_joint', self.motorDir[4]*kneeAngle)

    def getBasePosition(self):
        position, orientation = p.getBasePositionAndOrientation(self.ID)
        return position

    def getBaseOrientation(self):
        position, orientation = p.getBasePositionAndOrientation(self.ID)
        return orientation

    def applyAction(self, motorCommands):
        motorCommandsWithDir = np.multiply(motorCommands, self.motorDir)
        for i in range(self.nMotors):
            self.setMotorAngleById(
                self.motorIdList[i], motorCommandsWithDir[i])

    def getMotorAngles(self):
        motorAngles = []
        for i in range(self.nMotors):
            jointState = p.getJointState(self.ID, self.motorIdList[i])
            motorAngles.append(jointState[0])
        motorAngles = np.multiply(motorAngles, self.motorDir)
        return motorAngles

    def getMotorVelocities(self):
        motorVelocities = []
        for i in range(self.nMotors):
            jointState = p.getjointState(self.ID, self.motorIdList[i])
            motorVelocities.append(jointState[1])
        motorVelocities = np.multiply(motorVelocities, self.motorDir)
        return motorVelocities

    def getMotorTorques(self):
        motorTorques = []
        for i in range(self.nMotors):
            jointState = p.getJointState(self.ID, self.motorIdList[i])
            motorTorques.append(jointState[3])
        motorTorques = np.multiply(motorTorques, self.motorDir)
        return motorTorques

    def setRefFootsteps(self, ref_footsteps):
        self.ref_footsteps = ref_footsteps

    def walk(self, T_sup=0.8, z_c=0.8, a=10, b=1):
        if self.ref_footsteps is None:
            print("No footsteps")
            return


class PybulletSettings():
    def __init__(self):
        self.pysicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.setGravity(0, 0, -9.8)

        self.planeId = p.loadURDF("plane.urdf")
        p.setRealTimeSimulation(False)
        self.cameraConfig()

    # camera positioin setting
    def cameraConfig(self, distance=1, yaw=0, pitch=-30, targetPos=[0, 0, 0]):
        p.resetDebugVisualizerCamera(distance, yaw, pitch, targetPos)


if __name__ == "__main__":
    pybullet_settings = PybulletSettings()
    walker = BipedalPlanner("/urdf/lip_model01.urdf")

    for i in range(1000):
        p.stepSimulation()
        time.sleep(1./240.)
        if i == 300:
            walker.setMotorAngleByName('leftHip_joint', 1.)

    p.disconnect()
