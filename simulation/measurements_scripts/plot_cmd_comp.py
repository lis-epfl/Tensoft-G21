#! /usr/bin/env python3

# THIS IS EMPLOYED BY THE CABLES MEASUREMENTS SCRIPT

import os
import argparse
import numpy as np

import matplotlib
# select matplotlib backend
matplotlib.use('pdf')
import matplotlib.pyplot as plt

STEP_LENGTH = 10000

def plot_cmd_comp(data, out_dir, stiff=0.1, noise=0.0, use_absolute=False):
    font = {'family': 'Source Sans Pro', 'size': 18, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    fig = plt.figure(figsize=(14, 14))
    ax = fig.gca()

    labels = ['10 (1)', '9 (4)', '4 (6)', '7 (7)', '3 (10)']
    colors = plt.cm.viridis(np.linspace(0, 1, len(labels)))

    v = np.asarray([dv for dv in data.values()])

    if not use_absolute:
        for i in range(0, v.shape[1]):
            max_val = np.max(v[:, i])
            v[:, i] = v[:, i] / max_val

    ax.plot([], [], marker='*', label='Cables ID:', color='white')
    for i in range(len(labels)):
        ax.plot(data.keys(), v[:, i], 'o',
                label=labels[i],
                color=colors[i], ms=7)

    ax.set_title('Cable Length - Stiff: {} - Noise: {}'.format(stiff, noise),
                 size=26, fontweight='normal', pad=65)
    ax.set_xlabel('Motor command (deg)', labelpad=15, fontweight='light')
    ax.set_ylabel('{} Cable Length'.format('Absolute' if use_absolute else 'Relative'), labelpad=10, fontweight='light')
    ax.set_xticks(np.arange(0, 200, 20))
    ax.margins(x=0.02, y=0.0)
    # ax.set_xlim(-2, 182)
    if not use_absolute:
        ax.set_ylim(0.49, 1.01)
    else:
        ax.set_ylim(2, 5)
    # ax.legend(loc='center', borderaxespad=1, ncol=len(labels),
    #           frameon=False, bbox_to_anchor=(0.5, 1.03))
    ax.legend(loc='upper center', borderaxespad=-2.5, ncol=len(labels) + 1, frameon=False)
    ax.grid()
    plt.savefig(os.path.join(out_dir, 'cmd_compression_s_{}_n_{}.pdf'.format(stiff, noise)),
                bbox_inches='tight')
    plt.close()

    with open(os.path.join(out_dir, 'cmd_compression_s_{}_n_{}.csv'.format(stiff, noise)), 'w') as out_file:
        out_file.write('cycle,N1,N4,N6,N7,N10\n')
        for c, l in zip(data.keys(), v.tolist()):
            out_file.write(str(c) + ',' +','.join(map(str, l)) + '\n')


def main(data_file, out_dir=None, stiff=0.1, noise=0.0, use_absolute=False):
    with open(data_file) as in_file:
        in_file.readline()
        data = np.asarray([row.strip().split(',') for row in in_file], dtype='float32')

    if out_dir is not None:
        # create output folder if does not exists
        os.makedirs(out_dir, exist_ok=True)

    # remove measurements collected during the unstable phase
    measurements_data = data[(STEP_LENGTH * 5)-1:(STEP_LENGTH * 79) - 1]

    midpoint = measurements_data.shape[0] // 2
    end_p = measurements_data.shape[0]

    # prepare the data to be plotted, collecting all the data under
    # the same command (on the rising and falling edges of the signal)
    # and compute their mean along the columns (per cable length)
    data_per_cycle = {
        int(measurements_data[i+1][0]): np.median(np.vstack((measurements_data[i+1:i+STEP_LENGTH, 1:],
                                                             measurements_data[end_p-i-STEP_LENGTH:end_p-i, 1:])), axis=0)
        for i in range(0, midpoint, STEP_LENGTH)
    }

    plot_cmd_comp(data_per_cycle, out_dir, stiff, noise, use_absolute)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plots the command-cable_lengths visualization.')
    parser.add_argument('data_file', metavar='data_file', type=str,
                        help='the file where collected data are stored (csv)')
    parser.add_argument('out_dir', metavar='out_dir', type=str, default=None,
                        help='the folder where results should be stored')
    parser.add_argument('stiff', metavar='stiff', type=float, default=0.1, nargs='?',
                        help='the stiffness value of the tensegrity module')
    parser.add_argument('noise_level', metavar='noise_level', type=float, default=0.0, nargs='?',
                        help='the amount of noise introduced in the control signal')
    parser.add_argument('--absolute', dest='absolute', action='store_const',
                        const=True, help='use absolute values')
    args = parser.parse_args()

    main(args.data_file, args.out_dir, args.stiff, args.noise_level, args.absolute)