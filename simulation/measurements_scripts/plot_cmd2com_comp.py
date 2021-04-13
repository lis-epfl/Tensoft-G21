#! /usr/bin/env python3

# THIS IS EMPLOYED BY THE COM MEASUREMENTS SCRIPT

import os
import re
import argparse
import numpy as np

import matplotlib
# select matplotlib backend
matplotlib.use('pdf')
import matplotlib.pyplot as plt

def plot_cmd2comp(commands, data, out_dir, stiff_vals, is_relative=False):
    if is_relative:
        for dt in data:
            max_val = np.max(dt)
            for c in range(len(dt)):
                dt[c] = dt[c] / max_val

    # define chart settings
    font = {'family': 'Source Sans Pro', 'size': 14, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    color_shade = 6
    base_color = 0
    colors = plt.cm.viridis(np.linspace(0, 1, len(data)*color_shade+1))
    # colors = ['#23eb41', '#296be7', '#ff8502']
    # dark_colors = ['#0f3a8b', '#0d8b20', '#8b4800']

    fig = plt.figure(figsize=(16, 16))
    ax = fig.gca()
    fig_title = '{} Module Compression'.format('Relative' if is_relative else 'Absolute')
    ax.set_title(fig_title, size=28, fontweight='normal', pad=50)

    ax.plot([], ' ', label='Stiffness:', alpha=0)
    for i, dt in enumerate(data):
        ax.plot(commands, dt, '--',
                label=stiff_vals[i],
                color=colors[color_shade-base_color + i*color_shade], ms=7)

    ax.set_xlabel('Motor command (deg)', labelpad=15, size=20, fontweight='light')
    ax.set_ylabel('Module Compression', labelpad=10, size=20, fontweight='light')
    ax.set_xticks(np.arange(0, 200, 20))
    ax.margins(x=0.04, y=0.02)
    ax.set_ylim(np.floor(np.min(data)*10)/10 - 0.02, np.max(data) + 0.02)
    ax.grid()
    ax.legend(loc='center', borderaxespad=1, ncol=len(stiff_vals)+1,
              frameon=False, bbox_to_anchor=(0.48, 1.03))

    out_file = '{}_module_compression.pdf'.format('relative' if is_relative else 'absolute')
    plt.savefig(os.path.join(out_dir, out_file), bbox_inches='tight')
    plt.close()

def main(input_dir):
    data_files = [f for f in os.listdir(input_dir)
                  if re.match('cmd2comp_data_.+.txt', f)]

    data_per_stiff = []
    stiffness_vals = []
    for data_file in data_files:
        # extract stiffness value from filename
        stiffness_vals.append(data_file.split('_')[2][:-4])

        with open(os.path.join(input_dir, data_file)) as in_file:
            commands, raw_data = list(zip(*[row.strip().split(';') for row in in_file]))

        # preprocess input COMs data
        commands = list(map(int, commands))
        data = np.asarray([[p.split(' ') for p in points.split(',')]
                           for points in raw_data], dtype=np.float32)

        data_per_stiff.append([np.linalg.norm(np.mean(vs[[1, 3, 4]], axis=0) -
                                              np.mean(vs[[0, 2, 5]], axis=0))
                               for vs in data])

    # plot absolute compressions
    plot_cmd2comp(commands, data_per_stiff, input_dir, stiffness_vals)
    # plot relative compressions
    plot_cmd2comp(commands, data_per_stiff, input_dir, stiffness_vals, is_relative=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot command VS module compression diagram.')
    parser.add_argument('input_dir', metavar='input_dir', type=str,
                        help='the folder where collected data are stored')
    args = parser.parse_args()

    main(args.input_dir)