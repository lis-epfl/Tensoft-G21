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

sys.path.append('..')
from params_conf import MIN_NUM_MODULES, MAX_NUM_MODULES, N_MODULES, STIFF_TABLE


def load_archive(arch_file):
    archive = np.full(shape=(len(N_MODULES), len(STIFF_TABLE)), fill_value=np.NaN)
    with open(arch_file) as af:
        _ = af.readline() # skip header
        for line in af:
            nmod, stiff, fit = line.strip().split(',')
            nmod = int(nmod) - MIN_NUM_MODULES
            stiff = STIFF_TABLE.index(float(stiff))
            archive[nmod, stiff] = float(fit)

    return archive


def extract_features(robot_string):
    modules = robot_string.split('--')[:-1]
    # we consider only the first module since
    # all the modules have the same stiffness
    properties = modules[0].split('-')

    # last property of each module is its stiffness
    return len(modules) - MIN_NUM_MODULES, STIFF_TABLE.index(float(properties[-1]))


def load_hist_archive(hist_file):
    archive = np.full(shape=(len(N_MODULES), len(STIFF_TABLE)), fill_value=np.NaN)
    with open(hist_file) as hf:
        _ = hf.readline() # skip header
        for line in hf:
            robot_string, fit = line.strip().split(',')
            fit = float(fit)
            nmod, stiff = extract_features(robot_string)

            # add only the elites
            if np.isnan(archive[nmod, stiff]):
                archive[nmod, stiff] = fit
            elif archive[nmod, stiff] < fit:
                archive[nmod, stiff] = fit

    return archive


def load_archives(exp_folder, is_history=False):
    print('Loading archives...')
    if is_history:
        hist_files = [f for f in os.listdir(exp_folder)
                      if re.match('^history_[0-9]+_sim_[0-9]\.csv', f)]
        hist_files.sort(key=lambda f: int(f.split('_')[-1][0]))

        archives = [load_hist_archive(os.path.join(exp_folder, hf)) for hf in hist_files]
    else:
        arch_files = [f for f in os.listdir(exp_folder)
                      if re.match('^archive.*\.csv', f)]
        arch_files.sort(key=lambda f: int(f.split('_')[-1][0]))

        archives = [load_archive(os.path.join(exp_folder, af)) for af in arch_files]

    print('Plotting...')
    return archives


def plot_archives(archives, filename, min_fit=5, max_fit=None):
    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    fig, axes = plt.subplots(1, 5, sharey='all', figsize=(15, 10))

    for ax, arch in zip(axes, archives):
        ax.set_xlabel('Stiffness value', labelpad=7, fontweight='light', fontsize=16)
        ax.set_xticks(range(len(STIFF_TABLE)))
        ax.set_xticklabels(STIFF_TABLE)
        im = ax.imshow(arch, cmap='viridis', origin='lower', vmin=min_fit, vmax=max_fit)

    # set modules information
    axes[0].set_ylabel('# modules', labelpad=3, fontweight='light', fontsize=16)
    axes[0].set_yticks(range(len(N_MODULES)))
    axes[0].set_yticklabels(N_MODULES)

    # fix y ticks
    for ax in axes[1:]:
        ax.tick_params(axis='y', which='both', length=0)

    # color bar details
    im_ratio = archives[-1].shape[0] / archives[-1].shape[1]
    fig.subplots_adjust(right=0.97)
    cbar_ax = fig.add_axes([0.98, 0.3845, 0.01, im_ratio * 0.245])
    cbar = fig.colorbar(im, cax=cbar_ax)
    cbar.set_label('Fitness', rotation=270, labelpad=20, fontsize=16)

    plt.subplots_adjust(wspace=0.04)
    plt.savefig(filename, bbox_inches='tight')


def main(exp_folder, out_file, is_history=False, min_fit=5, max_fit=None):
    archives = load_archives(exp_folder, is_history=is_history)
    if len(archives) == 0:
        raise Exception('No archive has been loaded. Maybe you would'
                        'like to load histories with --history flag.')

    stats = { 'cov': [], 'mean': [], 'std': [], 'max': [], '_10': []}
    for i, a in enumerate(archives):
        print(a.dtype)
        stats['cov'].append((a.size - np.count_nonzero(np.isnan(a))) / a.size * 100)
        stats['_10'].append(np.count_nonzero(a <= 10) / a.size * 100)
        stats['mean'].append(np.nanmean(a))
        stats['std'].append(np.nanstd(a))
        stats['max'].append(np.nanmax(a))
        
        print(f'Archive {i} - mean: {stats["mean"][i]}, std: {stats["std"][i]},'
              f'max: {stats["max"][i]}, coverage: {stats["cov"][i]}%, under 10: {stats["_10"][i]}%')

    for k, v in stats.items():
        if k == 'max':
            print(k, np.max(v))
        else:
            print(k, np.mean(v))

    out_path = os.path.join(exp_folder, out_file)
    plot_archives(archives, out_path, min_fit, max_fit)
    print(f'Plot stored in {out_path}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Utility script for merging history files')
    parser.add_argument('exp_folder', metavar='exp_folder', type=str,
                        help='experiment folder containing the archive'
                             +'or history files - defaults to archives')
    parser.add_argument('--history', dest='is_history', action='store_const',
                        const=True, help='consider the input containing history files')
    parser.add_argument('--out-file', metavar='out-file', type=str, action='store',
                        default='archives_row.pdf', help='output filename where the plot is saved')
    parser.add_argument('--min-fit', metavar='min-fit', type=float, action='store',
                        default=5, help='minimum fitness value provided in the color bar')
    parser.add_argument('--max-fit', metavar='max-fit', type=float, action='store',
                        default=None, help='maximum fitness value provided in the color bar')
    args = parser.parse_args()

    main(args.exp_folder, args.out_file, args.is_history,
         min_fit=args.min_fit, max_fit=args.max_fit)
