import sys
import pybullet as p
import pybullet_data
from PIL import Image
import numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt
import seaborn as sns
sns.set()

sys.path.append('.')
sys.path.append('..')
from scripts.plot import *
from scripts.Jaco import Jaco


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


def main():
    Hz = 240
    ts = 1 / 240
    init_pybullet(Hz)

    robot = Jaco()

    # p.loadURDF('urdf/my_block.urdf', [-0.5, 0.5, 0.5])

    target_pos = [np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]

    frames = []
    position_list = []
    velocity_list = []
    torque_list = []
    torque_hat_list = []
    reactionForce_list = []
    target_pos_list = []
    input_list = []
    last_velocity = 0
    for t in tqdm(range(Hz * 20)):
        # pos = [-0.6 + 0.2 * np.cos(t / 240), 0.2 * np.sin(t / 240), 0.4]
        # robot.moveEndEffector(pos)
        # robot.setJointsAngle([
        #     np.pi + np.sin(t / 240),
        #     np.pi + np.sin(t / 240),
        #     np.pi + np.sin(t / 240),
        #     np.pi + np.sin(t / 240),
        #     np.pi + np.sin(t / 240),
        #     np.pi + np.sin(t / 240),
        #     np.pi + np.sin(t / 240),
        # ])
        position, velocity, torque, reactionForce = robot.getJointState()
        acceleration = (last_velocity - velocity) * ts
        input = target_pos - position + acceleration
        robot.setJointsVelocity(input)
        # last_velocity = velocity
        # print(p.getDynamicsInfo(robot.robotId, 2))
        # torque_hat = robot.calcID(position, velocity, acceleration)

        p.stepSimulation()

        if t % 24 == 0:
            width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                240, 240)
            frames.append(Image.fromarray(rgbImg))

        if t < 240:
            continue
        position_list.append(position)
        velocity_list.append(velocity)
        torque_list.append(torque)
        # torque_hat_list.append(torque_hat)
        reactionForce_list.append(reactionForce)
        target_pos_list.append(target_pos)
        input_list.append(input)

    position_list = np.array(position_list).transpose()
    velocity_list = np.array(velocity_list).transpose()
    torque_list = np.array(torque_list).transpose()
    torque_hat_list = np.array(torque_hat_list).transpose()
    reactionForce_list = np.array(reactionForce_list).transpose()
    target_pos_list = np.array(target_pos_list).transpose()

    save_gif(frames, 'result.gif')

    fig = plt.figure(figsize=(20, 20))
    plot_position(fig, target_pos_list, position_list)
    plt.savefig('position.png')

    fig = plt.figure(figsize=(20, 20))
    plot_torque(fig, velocity_list, velocity_list)
    plt.savefig('velocity.png')

    fig = plt.figure(figsize=(20, 20))
    plot_torque(fig, torque_list, torque_list)
    plt.savefig('torque.png')

    fig = plt.figure(figsize=(20, 20))
    plot_reaction_force(fig, reactionForce_list[3], reactionForce_list[3])
    plt.savefig('reaction_force.png')


if __name__ == '__main__':
    main()
