import numpy as np


def plot_state(fig, input, observation):
    dim = input.shape[0]
    col = 2
    row = int(dim / col) + 1

    for i in range(dim):
        ax = fig.add_subplot(row, col, i + 1)
        ax.plot(input[i], color='tab:gray', label='input')
        ax.plot(observation[i], color='tab:blue',
                alpha=0.5, label='observation')
        ax.set_title('joint {}'.format(i + 1))
        if i % col == 0:
            ax.set_ylabel('')
        if i >= dim - col:
            ax.set_xlabel('time [s]')
        else:
            ax.tick_params(bottom=False, labelbottom=False)
        if i == 0:
            ax.legend(loc='upper right')
    fig.align_labels()


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
