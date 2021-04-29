import pybullet as p
import pybullet_data
from PIL import Image
import numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt
import seaborn as sns
sns.set()


class Jaco():
    def __init__(self, pos=[0, 0, 0], rot=[0, 0, 0]):
        self.robotId = p.loadURDF(
            'urdf/kinova_description/urdf/j2n7s300.urdf',
            pos,
            p.getQuaternionFromEuler(rot))
        self.eeId = 9
        self.joint_num = p.getNumJoints(self.robotId)
        print('joints_num', self.joint_num)
        for i in range(self.joint_num):
            p.enableJointForceTorqueSensor(self.robotId, i)
            info = p.getJointInfo(self.robotId, i)
            print(i, info[1])
        self.reset()

    def calcIK(self, pos=[1, 1, 1], rot=[0, 0, 0]):
        return p.calculateInverseKinematics(self.robotId, self.eeId, pos, rot)

    def calcID(self, pos, vel, acc):
        return p.calculateInverseDynamics(self.robotId, pos, vel, acc)

    def reset(self):
        reset_point = self.calcIK([-0.5, -0.5, 0.5], [0, 0, 0])
        for i in range(2, self.eeId):
            p.resetJointState(self.robotId, i, reset_point[i])

    def moveEndEffector(self, pos=[1, 1, 1], rot=[0, 0, 0]):
        joint_angle = self.calcIK(pos, rot)
        self.setJointsAngle(joint_angle[:7])

    def setJointsAngle(self, jointAngle=[
                       np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]):
        p.setJointMotorControlArray(
            self.robotId,
            range(2, 9),
            p.POSITION_CONTROL,
            targetPositions=jointAngle,
        )

    def setJointsVelocity(self, jointVelocity=[0, 0, 0, 0, 0, 0, 0]):
        p.setJointMotorControlArray(
            self.robotId,
            range(2, 9),
            p.VELOCITY_CONTROL,
            targetVelocities=jointVelocity,
        )

    def setJointsTorque(self, jointTorque=[0, 0, 0, 0, 0, 0, 0]):
        p.setJointMotorControlArray(
            self.robotId,
            range(2, 9),
            p.TORQUE_CONTROL,
            force=jointTorque,
        )

    def getJointState(self):
        position_list = []
        velocity_list = []
        reactionForce_list = []
        torque_list = []
        for i in range(2, 9):
            position, velocity, reactionForce, torque = p.getJointState(
                self.robotId, i)
            position_list.append(position)
            velocity_list.append(velocity)
            reactionForce_list.append(reactionForce)
            torque_list.append(torque)

        position_list = np.array(position_list)
        velocity_list = np.array(velocity_list)
        torque_list = np.array(torque_list)
        reactionForce_list = np.array(reactionForce_list)
        return position_list, velocity_list, torque_list, reactionForce_list
