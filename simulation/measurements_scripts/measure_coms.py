#! /usr/bin/env python3

import os
import argparse
import subprocess
import numpy as np

import matplotlib
# select matplotlib backend
matplotlib.use('pdf')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

STEP_LENGTH = 10000

def get_stiffness(robot_file):
    with open(robot_file) as in_file:
        stiffness = in_file.readline().strip().split(' ')[-1]
    return stiffness


def plot_init_coms(med_data, face_coms, out_dir, stiff):
    # define chart settings
    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams['axes.titlepad'] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    colors = plt.cm.viridis(np.linspace(0, 1, med_data.shape[0]))
    labels = ['r(0-1)', 'r(2-3)', 'r(4-5)', 'r(6-7)', 'r(8-9)', 'r(10-11)']
    face_colors = ['skyblue', 'saddlebrown']
    face_labels = ['face 1', 'face 5']

    titles = ['Front', '3D', 'Top', 'Side']
    views = [(0, 0), (30, -60), (90, 0), (0, -90)]

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('Initial COM Positions - stiffness: {}'.format(stiff),
                 fontsize=20, fontweight='bold')

    # NOTE: chart axis have been rearranged into the position of ones managed within simulator

    ## TOP-LEFT SUBPLOT -> FRONT VIEW ##
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.set_title(titles[0], size=14, fontweight='normal')

    for i in range(med_data.shape[0]):
        ax1.scatter([med_data[i, 0]], [med_data[i, 1]],
                    marker='x', label=labels[i], color=colors[i])

    for j in range(face_coms.shape[0]):
        ax1.scatter([face_coms[j, 0]], [face_coms[j, 1]],
                    marker='^', label=face_labels[j], color=face_colors[j])

    ax1.plot(face_coms[:, 0], face_coms[:, 1],
             'black', alpha=0.7, linestyle='--')

    ax1.spines['right'].set_visible(False)
    ax1.spines['top'].set_visible(False)
    ax1.set_xlabel('X', labelpad=15)
    ax1.set_ylabel('Y', labelpad=10, rotation=0)

    ## TOP-RIGHT SUBPLOT -> 3D VIEW ##
    ax2 = fig.add_subplot(2, 2, 2, projection='3d')
    ax2.set_title(titles[1], size=14, fontweight='normal')

    for i in range(med_data.shape[0]):
        ax2.scatter([med_data[i, 2]], [med_data[i, 0]], [med_data[i, 1]],
                    marker='x', label=labels[i], color=colors[i])
    for j in range(face_coms.shape[0]):
            ax2.scatter([face_coms[j, 2]], [face_coms[j, 0]], [face_coms[j, 1]],
                        marker='^', label=face_labels[j], color=face_colors[j])

    ax2.plot3D(face_coms[:, 2], face_coms[:, 0], face_coms[:, 1],
               'black', alpha=0.7, linestyle='--')
    ax2.set_xlabel('Z', labelpad=15)
    ax2.set_ylabel('X', labelpad=10)
    ax2.set_zlabel('Y', labelpad=-2)

    ax2.view_init(elev=30, azim=-60)

    ## BOTTOM-LEFT SUBPLOT -> TOP VIEW ##
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.set_title(titles[2], size=14, fontweight='normal')

    for i in range(med_data.shape[0]):
        ax3.scatter([med_data[i, 0]], [med_data[i, 2]],
                    marker='x', label=labels[i], color=colors[i])

    for j in range(face_coms.shape[0]):
        ax3.scatter([face_coms[j, 0]], [face_coms[j, 2]],
                    marker='^', label=face_labels[j], color=face_colors[j])

    ax3.plot(face_coms[:, 0], face_coms[:, 2],
             'black', alpha=0.7, linestyle='--')
    ax3.invert_yaxis()
    ax3.spines['right'].set_visible(False)
    ax3.spines['top'].set_visible(False)
    ax3.set_xlabel('X', labelpad=15)
    ax3.set_ylabel('Z', labelpad=10, rotation=0)

    ## BOTTOM-LEFT SUBPLOT -> TOP VIEW ##
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.set_title(titles[3], size=14, fontweight='normal')

    for i in range(med_data.shape[0]):
        ax4.scatter([med_data[i, 1]], [med_data[i, 2]],
                    marker='x', label=labels[i], color=colors[i])

    for j in range(face_coms.shape[0]):
        ax4.scatter([face_coms[j, 1]], [face_coms[j, 2]],
                    marker='^', label=face_labels[j], color=face_colors[j])

    ax4.plot(face_coms[:, 1], face_coms[:, 2],
             'black', alpha=0.7, linestyle='--')

    ax4.invert_yaxis()
    ax4.spines['right'].set_visible(False)
    ax4.spines['top'].set_visible(False)
    ax4.set_xlabel('Y', labelpad=15)
    ax4.set_ylabel('Z', labelpad=10, rotation=0)

    # extract needed labels and add a white label to center
    handles, labels = ax1.get_legend_handles_labels()
    fig.legend(handles[:6], labels[:6],
               loc='upper center', borderaxespad=3, ncol=6, frameon=False)
    fig.legend(handles[-2:], labels[-2:],
               loc='upper center', borderaxespad=5, ncol=2, frameon=False)

    fig.subplots_adjust(wspace=0.16, hspace=0.42, top=0.84, bottom=0.2, right=0.85)
    plt.savefig(os.path.join(out_dir, 'coms_viz_{}_no_cables.pdf'.format(stiff)),
                bbox_inches='tight')


def plot_3d_coms(data, face_coms, out_dir, stiff):
    # define chart settings
    font = {'family': 'Source Sans Pro', 'size': 15, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams['axes.titlepad'] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    colors = plt.cm.viridis(np.linspace(0, 1, data[0].shape[0]))
    labels = ['r(0-1)', 'r(2-3)', 'r(4-5)', 'r(6-7)', 'r(8-9)', 'r(10-11)']
    face_colors = ['skyblue', 'saddlebrown']
    face_labels = ['face 1', 'face 5']

    titles = ['Front', '3D', 'Top', 'Side']

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('COM Compression - stiffness: {}'.format(stiff),
                 fontsize=25, fontweight='bold', y=1.03)

    # for legend organization
    lower_leg = []
    lower_lab = []

    # NOTE: chart axis have been rearranged into the position of ones managed within simulator

    ## TOP-LEFT SUBPLOT -> FRONT VIEW ##
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.set_title(titles[0], size=18, fontweight='normal')

    for k, vs in data.items():
        for j in range(vs.shape[0]):
            ax1.scatter([vs[j, 0]], [vs[j, 1]],
                        marker='x', label=labels[j], color=colors[j])

    for h in range(data[0].shape[0]):
        ax1.scatter([data[0][h, 0]], [data[0][h, 1]],
                    marker='*', color='red', s=10, label='Initial points')
        if h == 0:
            handles, labels = ax1.get_legend_handles_labels()
            lower_leg.append(handles[-1])
            lower_lab.append(labels[-1])

    for k, fvs in face_coms.items():
        for j in range(fvs.shape[0]):
            ax1.scatter([fvs[j, 0]], [fvs[j, 1]],
                        marker='^', label=face_labels[j], color=face_colors[j])

    for l in range(face_coms[0].shape[0]):
        ax1.scatter([face_coms[0][l, 0]], [face_coms[0][l, 1]],
                    marker='*', color='red', s=10)

    ax1.plot(face_coms[0][:, 0], face_coms[0][:, 1],
             'black', alpha=0.7, linestyle='--')
    ax1.plot(face_coms[180][:, 0], face_coms[180][:, 1],
             'blue', alpha=0.7, linestyle='--')
    ax1.spines['right'].set_visible(False)
    ax1.spines['top'].set_visible(False)
    ax1.set_xlabel('X', labelpad=15)
    ax1.set_ylabel('Y', labelpad=10, rotation=0)
    ax1.set_xlim(-2.4, 2.4)
    ax1.set_ylim(14.8, 19.2)

    ## TOP-RIGHT SUBPLOT -> 3D VIEW ##
    ax2 = fig.add_subplot(2, 2, 2, projection='3d')
    ax2.set_title(titles[1], size=16, fontweight='normal')

    for k, vs in data.items():
        for j in range(vs.shape[0]):
            ax2.scatter([vs[j, 2]], [vs[j, 0]], [vs[j, 1]],
                        marker='x', label=labels[j], color=colors[j])

    for h in range(data[0].shape[0]):
        ax2.scatter([data[0][h, 2]], [data[0][h, 0]], [data[0][h, 1]],
                    marker='*', color='red', s=10, label='Initial points')

    for k, fvs in face_coms.items():
        for j in range(fvs.shape[0]):
            ax2.scatter([fvs[j, 2]], [fvs[j, 0]], [fvs[j, 1]],
                        marker='^', label=face_labels[j], color=face_colors[j])

    for l in range(face_coms[0].shape[0]):
        ax2.scatter([face_coms[0][l, 2]], [face_coms[0][l, 0]], [face_coms[0][l, 1]],
                    marker='*', color='red', s=10)

    ax2.plot3D(face_coms[0][:, 2], face_coms[0][:, 0], face_coms[0][:, 1],
              'black', alpha=0.7, linestyle='--')
    ax2.plot3D(face_coms[180][:, 2], face_coms[180][:, 0], face_coms[180][:, 1],
              'blue', alpha=0.7, linestyle='--')
    ax2.set_xlabel('Z', labelpad=15)
    ax2.set_ylabel('X', labelpad=10)
    ax2.set_zlabel('Y', labelpad=3)
    ax2.set_zlim(14.8, 19.2)

    ax2.view_init(elev=30, azim=-60)

    ## BOTTOM-LEFT SUBPLOT -> TOP VIEW ##
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.set_title(titles[2], size=16, fontweight='normal')

    for k, vs in data.items():
        for j in range(vs.shape[0]):
            ax3.scatter([vs[j, 0]], [vs[j, 2]],
                        marker='x', label=labels[j], color=colors[j])

    for h in range(data[0].shape[0]):
        ax3.scatter([data[0][h, 0]], [data[0][h, 2]],
                    marker='*', color='red', s=10, label='Initial points')

    for k, fvs in face_coms.items():
        for j in range(fvs.shape[0]):
            ax3.scatter([fvs[j, 0]], [fvs[j, 2]],
                        marker='^', label=face_labels[j], color=face_colors[j])

    for l in range(face_coms[0].shape[0]):
        ax3.scatter([face_coms[0][l, 0]], [face_coms[0][l, 2]],
                    marker='*', color='red', s=10)

    ax3.plot(face_coms[0][:, 0], face_coms[0][:, 2],
             'black', alpha=0.7, linestyle='--')
    ax3.plot(face_coms[180][:, 0], face_coms[180][:, 2],
             'blue', alpha=0.7, linestyle='--')
    ax3.invert_yaxis()
    ax3.spines['right'].set_visible(False)
    ax3.spines['top'].set_visible(False)
    ax3.set_xlabel('X', labelpad=15)
    ax3.set_ylabel('Z', labelpad=10, rotation=0)
    ax3.set_xlim(-2.2, 2.2)
    ax3.set_ylim(-1.4, 4.2)

    ## BOTTOM-LEFT SUBPLOT -> TOP VIEW ##
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.set_title(titles[3], size=16, fontweight='normal')

    for k, vs in data.items():
        for j in range(vs.shape[0]):
            ax4.scatter([vs[j, 1]], [vs[j, 2]],
                        marker='x', label=labels[j], color=colors[j])

    for h in range(data[0].shape[0]):
        ax4.scatter([data[0][h, 1]], [data[0][h, 2]],
                    marker='*', color='red', s=10, label='Initial points')

    for k, fvs in face_coms.items():
        for j in range(fvs.shape[0]):
            ax4.scatter([fvs[j, 1]], [fvs[j, 2]],
                       marker='^', label=face_labels[j], color=face_colors[j])

    for l in range(face_coms[0].shape[0]):
        ax4.scatter([face_coms[0][l, 1]], [face_coms[0][l, 2]],
                    marker='*', color='red', s=10)

    ax4.plot(face_coms[0][:, 1], face_coms[0][:, 2],
              'black', alpha=0.7, linestyle='--')
    ax4.plot(face_coms[180][:, 1], face_coms[180][:, 2],
              'blue', alpha=0.7, linestyle='--')
    ax4.invert_yaxis()
    ax4.spines['right'].set_visible(False)
    ax4.spines['top'].set_visible(False)
    ax4.set_xlabel('Y', labelpad=15)
    ax4.set_ylabel('Z', labelpad=10, rotation=0)
    ax4.set_xlim(14.8, 19.2)
    ax4.set_ylim(-1.4, 4.2)

    # extract needed labels and add a white label to center
    handles, labels = ax1.get_legend_handles_labels()
    legend_h = handles[:6]
    labels_h = labels[:6]
    fig.legend(legend_h, labels_h,
               loc='upper center', borderaxespad=3, ncol=len(legend_h), frameon=False)

    lower_leg += handles[-2:]
    lower_lab += labels[-2:]
    fig.legend(lower_leg, lower_lab,
               loc='upper center', borderaxespad=5, ncol=len(lower_leg), frameon=False)

    fig.subplots_adjust(wspace=0.16, hspace=0.48, top=0.84, bottom=0.2, right=0.85)
    plt.savefig(os.path.join(out_dir, 'coms_viz_{}.pdf'.format(stiff)),
                bbox_inches='tight')


def extract_com(sim_output, out_dir, stiff, no_cables=False):
    print('Extracting data...')
    # extract from standard output the values (commands) generate by the NN
    raw_data = sim_output.stdout.decode('utf-8').strip().split('è,')[1:-1]

    filename = os.path.join(out_dir, 'coms_data_{}{}.csv'.format(stiff, '_no_cables' if no_cables else ''))
    with open(filename, 'w') as out_file:
        out_file.write('cycle,hr0,hr1,r2,r3,r4,r5,r6,lr\n')
        for l in raw_data:
            out_file.write(l + '\n')

    cycles = [float(line.split(',')[0]) for line in raw_data]
    data = np.asarray([[[float(d) for d in e.split(';')] for e in line.split(',')[1:]]
                       for line in raw_data])

    return cycles, data


def main(app, robot_conf, out_dir, no_active_cables):
    exec_string = '{} file {}'.format(app, robot_conf)
    # create output folder if does not exists
    os.makedirs(out_dir, exist_ok=True)

    print('Running simulation...')
    cproc = subprocess.run(exec_string.split(' '), capture_output=True)

    stiffness = get_stiffness(robot_conf)
    cycles, data = extract_com(cproc, out_dir, stiffness, no_active_cables)

    face_comps_file = 'face_comp_distance_{}{}.txt'.format(
        stiffness,
        '_no_cables' if no_active_cables else ''
    )

    print('Plotting results...')
    if no_active_cables:
        # prepare data (note -> 1:-1 removes redundant columns that contains the values for rod (0-1))
        med_data = np.median(data[-40000:, 1:-1, :], axis=0)

        face_com_1 = np.mean(med_data[[1, 3, 4]], axis=0)
        face_com_5 = np.mean(med_data[[0, 2, 5]], axis=0)
        face_coms = np.asarray([face_com_1, face_com_5])

        plot_init_coms(med_data, face_coms, out_dir, stiff=stiffness)

        face_comp_distance = np.linalg.norm(face_com_1-face_com_5)
        print('Initial "faces" distance:', face_comp_distance)

        with open(os.path.join(out_dir, face_comps_file), 'w') as out_file:
            out_file.write(str(face_comp_distance))

    else:
        # remove measurements collected during the unstable phase
        mes_cycles = cycles[(STEP_LENGTH * 5) - 1:(STEP_LENGTH * 79) - 1]
        measurements_data = data[(STEP_LENGTH * 5) - 1:(STEP_LENGTH * 79) - 1, 1:-1]

        midpoint = measurements_data.shape[0] // 2
        end_p = measurements_data.shape[0]

        # prepare the data to be plotted, collecting all the data under
        # the same command (on the rising and falling edges of the signal)
        # and compute their median along the columns (per cable length)
        data_per_cycle = {
            int(mes_cycles[i + 1]): np.median(
                np.vstack((measurements_data[i + 1:i + STEP_LENGTH, :],
                           measurements_data[end_p - i - STEP_LENGTH:end_p - i, :])), axis=0)
            for i in range(0, midpoint, STEP_LENGTH)
        }

        with open(os.path.join(out_dir, 'cmd2comp_data_{}.txt'.format(stiffness)), 'w') as out_file:
            for k, vs in data_per_cycle.items():
                out_array = ','.join(['{} {} {}'.format(*p) for p in vs])
                out_file.write('{};{}\n'.format(k, out_array))

        face_coms_per_cycle = {k: np.asarray([np.mean(vs[[1, 3, 4]], axis=0),
                                              np.mean(vs[[0, 2, 5]], axis=0)])
                               for k, vs in data_per_cycle.items()}

        plot_3d_coms(data_per_cycle, face_coms_per_cycle, out_dir, stiff=stiffness)

        face_comp_distances = []
        for k, fvs in face_coms_per_cycle.items():
            fc_val = np.linalg.norm(fvs[0] - fvs[1])
            face_comp_distances.append(fc_val)

            with open(os.path.join(out_dir, face_comps_file), 'w') as out_file:
                out_file.write(str(fc_val) + '\n')

        with open(os.path.join(out_dir, 'compressions_{}.txt'.format(stiffness)), 'w') as out_file:
            out_file.write('Initial "faces" distance: {}\n'.format(face_comp_distances[0]))
            out_file.write('Final "faces" distance: {}\n'.format(face_comp_distances[-1]))
            out_file.write('Absolute compression: {}\n'.format(face_comp_distances[0]-face_comp_distances[-1]))
            out_file.write('Relative compression: {}\n'.format(1 - face_comp_distances[-1]/face_comp_distances[0]))
        print('Initial "faces" distance:', face_comp_distances[0])
        print('Final "faces" distance:', face_comp_distances[-1])
        print('Absolute compression:', face_comp_distances[0]-face_comp_distances[-1])
        print('Relative compression:', 1 - face_comp_distances[-1]/face_comp_distances[0])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for measuring the center of mass module rods during the simulation.')
    parser.add_argument('sim_app', metavar='sim_app', type=str,
                        help='the program that simulate the robot')
    parser.add_argument('robot_file', metavar='robot_file', type=str,
                        help='the file where robot configuration is stored')
    parser.add_argument('out_dir', metavar='out_dir', type=str, default='../nn_commands',
                        help='the folder where results should be stored')
    parser.add_argument('--no_active_cables', dest='no_active_cables', action='store_const',
                        const=True, help='select whether to consider data from a module with no movements.')
    args = parser.parse_args()

    main(args.sim_app, args.robot_file, args.out_dir, args.no_active_cables)
