import pybullet as p
import numpy as np


class Jaco():
    def __init__(self, pos=[0, 0, 0], rot=[0, 0, 0]):
        self.robotId = p.loadURDF(
            'urdf/kinova_description/urdf/j2n7s300.urdf',
            pos,
            p.getQuaternionFromEuler(rot))
        self.eeId = 9
        self.joint_id_list = range(2, 9)
        self.joint_num = p.getNumJoints(self.robotId)
        print('joints_num', self.joint_num)
        for i in range(self.joint_num):
            p.enableJointForceTorqueSensor(self.robotId, i)
            info = p.getJointInfo(self.robotId, i)
            print(i, info[1])
        # self.reset_joint_angle = self.calcIK([-0.8, 0.0, 0.8])
        # self.reset_joint_angle = [np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]
        self.reset_joint_angle = [0, np.pi, 0, -np.pi/4, np.pi/2, 0, np.pi]
        self.reset(self.reset_joint_angle)
        self.M, self.D, self.J = self.getDynamicsInfos()
        print('M:', self.M)
        print('D:', self.D)
        print('J:', self.J)
        self.changeDynamics()
        self.M, self.D, self.J = self.getDynamicsInfos()
        print('M:', self.M)
        print('D:', self.D)
        print('J:', self.J)

    def getDynamicsInfo(self, jointId):
        mass, friction, inertia, pos, rot, _, _, _, _, _, _, _ = p.getDynamicsInfo(
            self.robotId, jointId)
        return mass, friction, inertia

    def getDynamicsInfos(self):
        M, D, J = [], [], []
        for i in self.joint_id_list:
            mass, friction, inertia = self.getDynamicsInfo(i)
            M.append(mass)
            D.append(friction)
            J.append(inertia[2])
        return M, D, J

    def changeDynamics(self):
        for i in range(len(self.joint_id_list)):
            mass = self.M[i] * np.random.normal(loc=1, scale=0.01)
            friction = self.D[i] * np.random.normal(loc=1, scale=0.01)
            p.changeDynamics(
                self.robotId,
                self.joint_id_list[i],
                mass,
                friction)

    def calcIK(self, pos=[1, 1, 1], rot=[np.pi/2, np.pi/2, np.pi/2]):
        jointAngle = p.calculateInverseKinematics(self.robotId, self.eeId, pos, rot)
        return jointAngle[:7]

    def calcID(self, pos, vel, acc):
        return p.calculateInverseDynamics(self.robotId, pos, vel, acc)

    def reset(self, jointAngle=[
            np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]):
        for i in range(len(self.joint_id_list)):
            p.resetJointState(self.robotId, self.joint_id_list[i], jointAngle[i])

    def moveEndEffector(self, pos=[1, 1, 1], rot=[0, 0, 0]):
        joint_angle = self.calcIK(pos, rot)
        self.setJointsAngle(joint_angle)

    def setJointsAngle(self, jointAngle=[
                       np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]):
        p.setJointMotorControlArray(
            self.robotId,
            self.joint_id_list,
            p.POSITION_CONTROL,
            targetPositions=jointAngle,
        )

    def setJointsVelocity(self, jointVelocity=[0, 0, 0, 0, 0, 0, 0]):
        p.setJointMotorControlArray(
            self.robotId,
            self.joint_id_list,
            p.VELOCITY_CONTROL,
            targetVelocities=jointVelocity,
        )

    def setJointsTorque(self, jointTorque=[0, 0, 0, 0, 0, 0, 0]):
        p.setJointMotorControlArray(
            self.robotId,
            self.joint_id_list,
            p.TORQUE_CONTROL,
            force=jointTorque,
        )

    def getJointState(self):
        position_list = []
        velocity_list = []
        reactionForce_list = []
        torque_list = []
        for i in self.joint_id_list:
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
