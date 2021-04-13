#! /usr/bin/env python3

import os
import re
import sys
import argparse

import matplotlib
import numpy as np
# select matplotlib backend
matplotlib.use('pdf')
import matplotlib.pyplot as plt


def load_trajectory(input_file, no_zero_origin=False):
    # move trajectories relative to plane origin
    X = []
    Z = []
    with open(input_file) as in_file:
        # skip header
        _ = in_file.readline()

        # choose trajectory origin
        if no_zero_origin:
            o_x = 0.0
            o_z = 0.0
        else:
            o_x, o_z = in_file.readline().strip().split(',')
            o_x = float(o_x)
            o_z = float(o_z)
            X.append(float(o_x) - o_x)
            Z.append(float(o_z) - o_z)

        for i, line in enumerate(in_file):
            line = line.strip()
            if len(line) > 0:
                x, z = line.split(',')
                X.append(float(x) - o_x)
                Z.append(float(z) - o_z)

    return X, Z


def load_trajectories(folder, num_runs=5):
    trj_folder = os.path.join(folder, 'best', 'trajectories')
    trajectories = {k: [] for k in range(num_runs)}
    for f in os.listdir(trj_folder):
        if re.match('^robot_.*\.csv', f):
            t = load_trajectory(os.path.join(trj_folder, f))
            trajectories[int(f.split('_')[-1][0])].append(t)

    return trajectories


def plot_trajectories(trajectories_exp, filepath, plane_size, num_robots=10):
    # plot it to a file
    font = {'family': 'Source Sans Pro', 'size': 11, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 200

    colors = plt.cm.viridis(np.linspace(0, 1, num_robots))
    fig, axes = plt.subplots(1, 5, sharey='all', figsize=(21, 9))

    ticks = list(range(0, -int(plane_size), -15)) + list(range(0, int(plane_size), 15))
    for ax, trajectories in zip(axes, trajectories_exp.values()):
        for i, t in enumerate(trajectories):
            ax.plot(t[0], t[1], '-',color=colors[i])

        ax.set_xlim(-plane_size, plane_size)
        ax.set_ylim(-plane_size, plane_size)
        ax.set_aspect('equal', 'box')
        ax.set_xticks(ticks)
        ax.set_yticks(ticks)
        ax.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=True)
        ax.tick_params(axis='y', which='both', left=False, right=False, labelleft=False)

    axes[0].tick_params(axis='y', which='both', left=False, right=False, labelleft=True)
    # axes[0].set_ylabel('MAP-Elites', labelpad=5, fontweight='light', fontsize=20)
    plt.subplots_adjust(wspace=0.0)
    plt.savefig(filepath, bbox_inches='tight')


def main(exp_folder, out_file, plane_size):
    print('Loading trajectories...')
    exp_trjs = load_trajectories(exp_folder)

    max_travel = 0
    for k, v in exp_trjs.items():
        for x, z in v:
            max_travel = max(max_travel, max(np.abs(x)), max(np.abs(z)))
    if plane_size is None or plane_size < max_travel:
        plane_size = np.ceil(max_travel)+1

    print('Plotting trajectories...')
    out_path = os.path.join(exp_folder, out_file)
    plot_trajectories(exp_trjs, out_path, plane_size)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Utility script for merging history files')
    parser.add_argument('exp_folder', metavar='exp_folder', type=str,
                        help='experiment folder')
    parser.add_argument('--out-file', metavar='out-file', type=str, action='store',
                        default='trajectories_runs.pdf',
                        help='output filename where the plot is stored')
    parser.add_argument('--plane-size', metavar='plane-size', type=float, action='store',
                        default=None, help='half size of the plane where the robot moves')
    args = parser.parse_args()

    main(args.exp_folder, args.out_file, args.plane_size)
