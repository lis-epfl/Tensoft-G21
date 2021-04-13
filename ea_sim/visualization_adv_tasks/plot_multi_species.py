import argparse
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import os
import re
import subprocess
import sys

from matplotlib import gridspec

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from goal_reaching.statistics_reporter import StatisticsReporterGR


def load_stats_from_checkpoint_file(i, stats_checkpoint_file):
    print('\tRun #{}'.format(i))
    most_fit_genomes, generation_statistics = StatisticsReporterGR.restore_checkpoint(stats_checkpoint_file)
    statistics = StatisticsReporterGR(most_fit_genomes=most_fit_genomes,
                                      generation_statistics=generation_statistics)

    return statistics


def load_stats(exp_folder):
    stats_checkpoint_files = [os.path.join(exp_folder, f)
                              for f in os.listdir(exp_folder)
                              if re.match('contr_stats_cp_[0-9]+_[0-9]+_[0-9]+', f)]
    stats_checkpoint_files.sort(key=lambda f: f.split('_')[-1])

    stats = [
        load_stats_from_checkpoint_file(i, stats_checkpoint_file)
        for (i, stats_checkpoint_file) in enumerate(stats_checkpoint_files)
    ]

    return stats


def plot_multi_species(conf):
    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    fig = plt.figure(figsize=(15, 10))
    fig.suptitle(conf['title'], y=0.92, fontweight='normal', fontsize=18)

    gs = fig.add_gridspec(2, 6)
    axes = [
        plt.subplot(gs[0, :2]),
        plt.subplot(gs[0, 2:4]),
        plt.subplot(gs[0, 4:6]),
        plt.subplot(gs[1, 1:3]),
        plt.subplot(gs[1, 3:5])
    ]

    for ax, (i, stats) in zip(axes, enumerate(conf['statistics'])):
        species_sizes = stats.get_species_sizes()
        num_generations = len(species_sizes)
        curves = np.array(species_sizes).T

        ax.stackplot(range(num_generations), *curves)
        ax.set_xlabel(conf['x_labels'][i])

    # set y axis information
    axes[0].set_ylabel(conf['y_label'])
    axes[3].set_ylabel(conf['y_label'])

    # fix y ticks
    for ax in (axes[1:3]+[axes[4]]):
        ax.tick_params(axis='y', which='both', length=0)
        ax.tick_params(axis='y', label1On=False)

    plt.subplots_adjust(wspace=0.04)
    plt.savefig(conf['out_file'], bbox_inches='tight')
    plt.close()


def main(exp_folder, parent_folders_num, out_file, labels, owner):
    print('Loading files...')
    statistics = load_stats(exp_folder)

    parent_folder = os.path.normpath(exp_folder)
    for i in range(0, parent_folders_num):
        parent_folder = os.path.dirname(parent_folder)
    out_folder = os.path.join(parent_folder, 'evo_plots')
    os.makedirs(out_folder, exist_ok=True)

    if labels is None:
        labels = ['Run #{} - Generations'.format(i) for i in range(0, len(statistics))]
    elif len(labels) != len(statistics):
        raise Exception('The number of provided labels'
                        + 'is different from the number of speciation checkpoint files')

    print('Plotting results...')
    conf = {
        'statistics': statistics,
        'title': 'Speciation throughout generations',
        'x_labels': labels,
        'y_label': 'Size per Species',
        'out_file': os.path.join(out_folder, out_file)
    }
    plot_multi_species(conf)

    if owner is not None and len(owner) == 2:
        try:
            exec_string = 'chown -R {}:{} {}'.format(owner[0], owner[1], out_folder)
            c_proc = subprocess.run(exec_string.split(' '), capture_output=True)
        except:
            raise Exception('An error occurred during the owner setting')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for generating speciation plots for multiple runs')
    parser.add_argument('exp_folder', metavar='exps_folder', type=str, nargs='?',
                        help='experiment folder containing speciation checkpoint files '
                             '(contr_cp_[seed]_[gen]_[run])')
    parser.add_argument('--parent-folders-num', metavar='parent-folders-num', type=int, action='store',
                        default=1, help='number of parent folders to traverse before creating the out folder')
    parser.add_argument('--out-file', metavar='out-file', type=str, action='store',
                        default='speciation.pdf', help='output filename')
    parser.add_argument('--labels', metavar='labels', type=str, action='store', nargs='+',
                        default=None, help='charts labels - must match the number of speciation files!')
    parser.add_argument('--owner', metavar='owner', type=str, action='store', nargs='+',
                        default=None, help='User and group for chown')

    args = parser.parse_args()
    main(args.exp_folder, args.parent_folders_num, args.out_file, args.labels, args.owner)
