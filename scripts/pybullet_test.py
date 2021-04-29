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
from scripts.Jaco import Jaco
from scripts.plot import *


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
    ts = 1 / Hz
    init_pybullet(Hz)

    robot = Jaco()

    # p.loadURDF('urdf/my_block.urdf', [-0.5, 0.5, 0.5])

    # target_pos = [np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]

    frames = []
    position_list = []
    velocity_list = []
    torque_list = []
    torque_hat_list = []
    reactionForce_list = []
    target_pos_list = []
    target_vel_list = []
    input_list = []
    last_velocity = 0
    for t in tqdm(range(Hz * 6)):
        position, velocity, torque, reactionForce = robot.getJointState()

        if t < Hz * 2:
            target_pos = robot.reset_joint_angle
        elif Hz * 2 <= t < Hz * 4:
            # target_pos = robot.calcIK(pos=[0.0, 0.0, 0.8])
            target_pos = [np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]
        else:
            # target_pos = robot.calcIK(pos=[-0.5, 0.5, 0.5])
            target_pos = [np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]

        # pos = [-0.6 + 0.2 * np.cos(t / 240), 0.2 * np.sin(t / 240), 0.4]
        # pos = [-0.6 + 0.2 * np.cos(t / Hz), 0.2 * np.sin(t / Hz), 0.4]
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

        acceleration = (last_velocity - velocity) * ts
        input = target_pos - position + acceleration
        robot.setJointsVelocity(input)
        last_velocity = velocity

        p.stepSimulation()

        if t % int(Hz / 50) == 0:
            width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                240, 240)
            frames.append(Image.fromarray(rgbImg))

        if t < Hz:
            continue
        position_list.append(position)
        velocity_list.append(velocity)
        torque_list.append(torque)
        # torque_hat_list.append(torque_hat)
        reactionForce_list.append(reactionForce)
        target_pos_list.append(target_pos)
        target_vel_list.append(input)
        input_list.append(input)

    position_list = np.array(position_list).transpose()
    velocity_list = np.array(velocity_list).transpose()
    torque_list = np.array(torque_list).transpose()
    torque_hat_list = np.array(torque_hat_list).transpose()
    reactionForce_list = np.array(reactionForce_list).transpose()
    target_pos_list = np.array(target_pos_list).transpose()
    target_vel_list = np.array(target_vel_list).transpose()

    save_gif(frames, 'result.gif')

    fig = plt.figure(figsize=(20, 20))
    plot_state(fig, target_pos_list, position_list)
    plt.savefig('position.png')

    fig = plt.figure(figsize=(20, 20))
    plot_state(fig, target_vel_list, velocity_list)
    plt.savefig('velocity.png')

    fig = plt.figure(figsize=(20, 20))
    plot_torque(fig, torque_list, torque_list)
    plt.savefig('torque.png')

    fig = plt.figure(figsize=(20, 20))
    plot_reaction_force(fig, reactionForce_list[3], reactionForce_list[3])
    plt.savefig('reaction_force.png')


if __name__ == '__main__':
    main()
