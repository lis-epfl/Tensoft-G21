#! /usr/bin/env python3

import os
import re
import math
import argparse
import numpy as np

import matplotlib
# select matplotlib backend
matplotlib.use('pdf')
import colorsys
import matplotlib.pyplot as plt
import matplotlib.colors as mc

from pathlib import Path

def darken_color(color, amount):
    try:
        c = mc.cnames[color]
    except:
        c = color
    c = colorsys.rgb_to_hls(*mc.to_rgb(c))
    return colorsys.hls_to_rgb(c[0], 1 - amount * (1 - c[1]), c[2])

def load_trajectory(input_file, no_zero_origin=False):
    # move trajectories relative to plane origin
    trjs = { 'X': [], 'Z': [], 'x0': [], 'z0': [], 'xM': [], 'zM': [] }
    with open(input_file) as in_file:
        # skip header
        _ = in_file.readline()

        # choose trajectory origin
        for line in in_file:
            line = line.strip()
            if len(line) > 0:
                points = line.split(',')

                for t, o in zip(trjs.keys(), points):
                    trjs[t].append(float(o))

        if not no_zero_origin:
            # center the trajectory in the plane origin for visualization purposes
            o_x = trjs['X'][0]
            o_z = trjs['Z'][0]
            for i, (t_k, t_points) in enumerate(trjs.items()):
                if i % 2 == 0:
                    trjs[t_k] = [p-o_x for p in t_points]
                else:
                    trjs[t_k] = [p - o_z for p in t_points]

    return trjs


def plot_trajectories(trajectories, robots_name, filepath, sim_id=None, show_act=False):
    padding = 3

    # plot it to a file
    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 200

    colors = plt.cm.viridis(np.linspace(0, 1, len(trajectories)))

    fig = plt.figure(figsize=(12, 12))
    ax = fig.gca()
    ax.set_title('Robot Trajectory - Distance in [cm]- Sim {}'.format(sim_id),
                 size=20, fontweight='normal', pad=55)

    # define ground plane size
    lim = 0
    for trj in trajectories:
        for points in trj.values():
            if len(points) > 0:
                lim = max(lim, math.ceil(np.nanmax(np.abs(points))))
    lim += padding
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)

    for i, t_vals in enumerate(trajectories):
        t = list(t_vals.values())
        string_comp = Path(robots_name[i]).stem.split('_')
        label = '#mod {} stiff {}'.format(string_comp[2], string_comp[3])
        ax.plot(t[0], t[1], '-', label=label, color=colors[i])
        # initial points
        ax.plot(t[0][0], t[1][0], 'o', color='blue', ms=5)

        if show_act:
            ax.plot(t[2], t[3], '--', color=colors[i], linewidth=1)
            ax.plot(t[4], t[5], ':', color=colors[i], linewidth=1)
            # initial points
            ax.plot(t[2][0], t[3][0], '*', color=darken_color(colors[i], 1.2), ms=5)
            ax.plot(t[4][0], t[5][0], 'd', color=darken_color(colors[i], 1.2), ms=5)

    ax.legend(loc='upper center', borderaxespad=-3.7, ncol=5, frameon=False)

    plt.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=True)
    plt.tick_params(axis='y', which='both', left=False, right=False, labelleft=True)
    plt.savefig(filepath, bbox_inches='tight')


def plot_trajectories_subfig(trajectories, robots_name, filepath, sim_id=None, show_act=False):
    padding = 3

    # define ground plane size
    plane_size = 0
    for trj in trajectories:
        for points in trj.values():
            if len(points) > 0:
                plane_size = max(plane_size, math.ceil(np.nanmax(np.abs(points))))
    plane_size += padding

    # plot it to a file
    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 200

    colors = plt.cm.viridis(np.linspace(0, 1, 10))


    num_planes = len(trajectories)

    if num_planes <= 2:
        rows = 1
        columns = 2
    elif num_planes % 3 == 0:
        rows = int(np.ceil(num_planes / 3))
        columns = 3
    elif num_planes % 4 == 0 or num_planes < 10:
        rows = int(np.ceil(num_planes / 4))
        columns = 4
    else:
        rows = int(np.ceil(num_planes / 5))
        columns = 5

    fig, axes = plt.subplots(rows, columns, sharey='all', figsize=(20, 10))
    fig.suptitle('Robots Trajectory - Distance in [cm]- Sim {}'.format(sim_id),
                     size=20, fontweight='normal')

    if rows == 1:
        axes = axes.reshape((rows, columns))

    ticks = list(range(0, -int(plane_size), -15)) + list(range(0, int(plane_size), 15))

    h = 0
    for i, t_vals in enumerate(trajectories):
        ax = axes[i//columns, i%columns]
        t = list(t_vals.values())
        string_comp = Path(robots_name[i]).stem.split('_')
        ax.set_title('{} modules - stiff {}'.format(string_comp[2], string_comp[3]), size=13, pad=-22)
        ax.plot(t[0], t[1], '-', color=colors[0])
        # initial points
        ax.plot(t[0][0], t[1][0], 'o', color='blue', ms=3)

        if show_act:
            ax.plot(t[2], t[3], '--', color=colors[3], linewidth=1)
            ax.plot(t[4], t[5], ':', color=colors[7], linewidth=1)
            # initial points
            ax.plot(t[2][0], t[3][0], '*', color=darken_color(colors[3], 1.2), ms=5)
            ax.plot(t[4][0], t[5][0], 'd', color=darken_color(colors[7], 1.2), ms=5)

        ax.set_xlim(-plane_size, plane_size)
        ax.set_ylim(-plane_size, plane_size)
        ax.set_aspect('equal', 'box')
        ax.set_xticks(ticks)
        ax.set_yticks(ticks)
        ax.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)
        ax.tick_params(axis='y', which='both', left=False, right=False, labelleft=False)
        h = i

    for j in range(h%len(axes[-1])+1, len(axes[-1])):
        axes[-1, j].set_axis_off()

    for r in range(rows):
        axes[r, 0].tick_params(axis='y', which='both', left=False, right=False, labelleft=True)

    # add bottom labels only in the last row
    for ax in axes[-1]:
        ax.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=True)

    plt.subplots_adjust(wspace=0.0, hspace=0.005)
    plt.savefig(filepath, bbox_inches='tight')


def main(input_files, output_path, no_zero_origin=False, show_act=False):
    out_dir = os.path.dirname(input_files[0])
    out_f = os.path.join(out_dir, output_path)

    trajectories = [load_trajectory(t_file, no_zero_origin)
                    for t_file in input_files]
    plot_trajectories(trajectories, input_files, out_f, show_act=show_act)

    print(f'Trajectories plot saved in {out_f}')


def from_folder(input_folder, output_file, sim_id=0, no_zero_origin=False, show_act=False):
    trajectories_files = [f for f in os.listdir(input_folder)
                    if re.match('robot.*{}.csv'.format(sim_id), f)]

    if len(trajectories_files) == 0:
        raise Exception('No file found to match expected format: robot.*{}.csv')

    plot_trajectories_subfig([load_trajectory(os.path.join(input_folder, tf), no_zero_origin)
                       for tf in trajectories_files],
                      trajectories_files,
                      os.path.join(input_folder, output_file),
                      sim_id, show_act=show_act)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for plotting the robots trajectory')
    parser.add_argument('trajectory_file', metavar='trajectory_file', type=str, nargs='*',
                        default='trajectory.csv',
                        help='list of files that contains robot trajectory')
    parser.add_argument('--input-folder', metavar='input-folder', type=str, action='store',
                        default=None, help='input folder where trajectories are stored')
    parser.add_argument('--output-file', metavar='output-file', type=str, action='store',
                        default='trajectories.pdf', help='filename where to write the trajectories plot')
    parser.add_argument('--no-zero-origin', dest='no_zero_origin', action='store_const',
                        const=True, help='disable trajectory relocation to origin')
    parser.add_argument('--sim-id', metavar='sim-id', type=str, action='store',
                        default=0, help='select from which simulation plot the trajectories')
    parser.add_argument('--show-actuators', dest='show_actuators', action='store_const',
                        const=True, help='enable the visualization of first and last modules')

    args = parser.parse_args()

    if args.input_folder is not None and os.path.exists(args.input_folder):
        from_folder(args.input_folder, args.output_file, args.sim_id,
                    no_zero_origin=args.no_zero_origin, show_act=args.show_actuators)
    elif len(args.trajectory_file) > 0:
        main(args.trajectory_file, args.output_file,
             args.no_zero_origin, show_act=args.show_actuators)
    else:
        raise Exception('No trajectory file has been provided! Run with -h to program instructions.')
