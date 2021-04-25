import pybullet as p
import pybullet_data
from PIL import Image
import numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt
import seaborn as sns
sns.set()


def save_gif(frames, path):
    frames[0].save(path,
                   save_all=True,
                   append_images=frames[1:],
                   duration=20,
                   loop=0)


def init_pybullet(Hz=240):
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    timestep = 1 / Hz
    p.setTimeStep(timestep)
    p.loadURDF('plane.urdf')


def plot_torque(fig, y, pred):
    y = np.array(y)
    pred = np.array(pred)
    # t = np.arange(len(y)) * 0.002
    dim = y.shape[0]
    col = 2
    row = int(dim / col) + 1

    for i in range(dim):
        ax = fig.add_subplot(row, col, i + 1)
        # ax.plot(y[i], color='tab:gray', label='Ground Truth')
        ax.plot(pred[i], color='tab:blue',
                alpha=0.5, label='Estimated Value')
        ax.set_title('joint {}'.format(i + 1))
        if i % col == 0:
            ax.set_ylabel('torque [Nm]')
        if i >= dim - col:
            ax.set_xlabel('time [s]')
        else:
            ax.tick_params(bottom=False, labelbottom=False)
        if i == 0:
            ax.legend(loc='upper right')
    fig.align_labels()


def plot_reaction_force(fig, y, pred):
    y = np.array(y)
    pred = np.array(pred)
    # t = np.arange(len(y)) * 0.002
    dim = y.shape[0]
    col = 2
    row = int(dim / col) + 1

    for i in range(dim):
        ax = fig.add_subplot(row, col, i + 1)
        # ax.plot(y[i], color='tab:gray', label='Ground Truth')
        ax.plot(pred[i], color='tab:blue',
                alpha=0.5, label='Estimated Value')
        ax.set_title('joint {}'.format(i + 1))
        if i % col == 0:
            ax.set_ylabel('reaction force [Nm]')
        if i >= dim - col:
            ax.set_xlabel('time [s]')
        else:
            ax.tick_params(bottom=False, labelbottom=False)
        if i == 0:
            ax.legend(loc='upper right')
    fig.align_labels()


class Jaco():
    def __init__(self, pos=[0, 0, 0], rot=[0, 0, 0]):
        self.robotId = p.loadURDF(
            'urdf/kinova_description/urdf/j2n7s300.urdf',
            # 'urdf/crane_x7_description/urdf/crane_x7.urdf',
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

    def reset(self):
        reset_point = self.calcIK([-0.5, -0.5, 0.5], [0, 0, 0])
        for i in range(2, self.eeId):
            p.resetJointState(self.robotId, i, reset_point[i])

    def move_end_effector(self, pos=[1, 1, 1], rot=[0, 0, 0]):
        joint_poses = self.calcIK(pos, rot)
        for i in range(2, self.eeId):
            p.setJointMotorControl2(
                self.robotId,
                i,
                p.POSITION_CONTROL,
                targetPosition=joint_poses[i],
                targetVelocity=0,
            )

    def move_joints(self, jointAngle=[np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]):
        for i in range(7):
            p.setJointMotorControl2(
                self.robotId,
                i + 2,
                p.POSITION_CONTROL,
                targetPosition=jointAngle[i],
                targetVelocity=0,
            )

    def stop(self):
        for i in range(self.joint_num):
            p.setJointMotorControl2(
                self.robotId,
                i,
                p.VELOCITY_CONTROL,
                targetVelocity=0,
            )

    def getState(self):
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
        return np.concatenate(
            [position_list, velocity_list, torque_list]), reactionForce_list


def main():
    init_pybullet()

    robot = Jaco()

    p.loadURDF('urdf/my_block.urdf', [-0.5, 0.5, 0.5])

    frames = []
    state_list = []
    reactionForce_list = []
    for t in tqdm(range(240 * 20)):
        # pos = [-0.6 + 0.2 * np.cos(t / 240), 0.2 * np.sin(t / 240), 0.4]
        # robot.move_end_effector(pos)
        robot.move_joints([
            np.pi + np.cos(t/240),
            np.pi + np.cos(t/240),
            np.pi + np.cos(t/240),
            np.pi + np.cos(t/240),
            np.pi + np.cos(t/240),
            np.pi + np.cos(t/240),
            np.pi + np.cos(t/240),
        ])

        p.stepSimulation()

        if t % 24 == 0:
            width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                # 600, 600)
                240, 240)
            frames.append(Image.fromarray(rgbImg))

        if t < 240:
            continue
        state, reactionForce = robot.getState()
        state_list.append(state)
        reactionForce_list.append(reactionForce)

    state_list = np.array(state_list).transpose()
    reactionForce_list = np.array(reactionForce_list).transpose()
    # print(reactionForce_list.shape)
    # print(reactionForce_list)

    save_gif(frames, 'result.gif')

    fig = plt.figure(figsize=(20, 20))
    plot_torque(fig, state_list[14:21], state_list[14:21])
    plt.savefig('torque.png')

    fig = plt.figure(figsize=(20, 20))
    plot_reaction_force(fig, reactionForce_list[3], reactionForce_list[3])
    plt.savefig('reaction_force.png')


if __name__ == '__main__':
    main()
