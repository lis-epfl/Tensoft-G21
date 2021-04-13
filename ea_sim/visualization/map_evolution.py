#! /usr/bin/env python3

import os
import re
import sys
import argparse
import matplotlib
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from map_like import heatmap, heatmap_3d
from params_conf import N_MODULES, MIN_NUM_MODULES, MAX_NUM_MODULES, STIFF_TABLE


def load_archive(file, params, regex):
    data = []
    with open(file, 'r') as archive_file:
        _ = archive_file.readline()
        for line in archive_file:
            mods, stiff, fit = line.strip().split(',')
            data.append([int(mods)-MIN_NUM_MODULES, str(stiff), float(fit)])

    arch = np.full(shape=(len(N_MODULES), len(STIFF_TABLE)), fill_value=np.NaN)
    for r in data:
        arch[r[0], params['inv_stiff_table'][r[1]]] = r[2]

    return int(regex.findall(os.path.basename(file))[0]), arch

def load_archive_3d(file, inv_stiff_table, regex):
    data = []
    with open(file, 'r') as archive_file:
        _ = archive_file.readline()
        for line in archive_file:
            mods, stiff_min, stiff_max, fit = line.strip().split(',')
            stiff_min = float(stiff_min)
            stiff_max = float(stiff_max)
            data.append([int(mods)-MIN_NUM_MODULES,
                         str(int(stiff_min) if stiff_min.is_integer() else stiff_min),
                         str(int(stiff_max) if stiff_max.is_integer() else stiff_max),
                         float(fit)])

    d1 = len(N_MODULES)
    d2 = d3 = len(STIFF_TABLE)
    arch = np.full(shape=(d3, d1 * d2), fill_value=np.NaN)
    for r in data:
        second_idx = inv_stiff_table[r[1]] * d1 + r[0]
        arch[inv_stiff_table[r[2]], second_idx] = r[3]

    return int(regex.findall(os.path.basename(file))[0]), arch


def arch_evolution(archives, fps=10, out_folder='', max_fit=None):
    font = {'family': 'Source Sans Pro', 'size': 16, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    fig = plt.figure(figsize=(12.44, 7))
    ax = fig.gca()
    ax.set_title('MAP-Elites Features Space', size=24, fontweight='normal')
    ax.set_xlabel('Stiffness value', labelpad=10, fontweight='light')
    ax.set_ylabel('# modules', labelpad=10, fontweight='light')

    ashape = archives[0].shape
    if max_fit is None:
        max_fit = np.nanmax(archives[-1])

    # initialize the writer
    ffmpeg_w = FFMpegWriter(fps=fps, codec='libx264', bitrate=-1)

    with ffmpeg_w.saving(fig, os.path.join(out_folder, 'map_evo.mp4'), dpi=128):
        im = ax.imshow(np.full(shape=ashape, fill_value=np.NaN), cmap='viridis',
                       vmin=0, vmax=max_fit, aspect='auto', animated=True)

        im_ratio = ashape[0] / ashape[1]
        cbar = fig.colorbar(im, ax=ax, fraction=im_ratio * 0.047, pad=0.04)
        cbar.set_label('Fitness', rotation=270, labelpad=20)

        if STIFF_TABLE is not None:
            plt.xticks(range(len(STIFF_TABLE)), STIFF_TABLE)
        if MAX_NUM_MODULES is not None and MIN_NUM_MODULES is not None:
            plt.yticks(range(MAX_NUM_MODULES - MIN_NUM_MODULES + 1),
                       range(MIN_NUM_MODULES, MAX_NUM_MODULES + 1))

        ffmpeg_w.grab_frame()
        num_frames = len(archives)
        for f, archive in enumerate(archives):
            print('\rProcessed frame: {}/{}'.format(f, num_frames), end='')
            ax.imshow(archive, cmap='viridis', vmin=0, vmax=max_fit,
                      aspect='auto', animated=True)
            ffmpeg_w.grab_frame()
    print('\nAnimation saved!')


def single_frame(results_folder, archive_file, max_fit=None,
                 is_noisy=False, is_3d=False):
    inv_stiff_table = {
        str(k): v for k, v in zip(STIFF_TABLE, range(len(STIFF_TABLE)))
    }
    arch_id_rule = re.compile('[0-9]+')

    title = 'MAP-Elites Features Space - {}'.format('Noisy' if is_noisy else 'Noiseless')
    out_name = os.path.join(results_folder, archive_file.split('.')[0] + '.pdf')

    if is_3d:
        _, arch = load_archive_3d(os.path.join(results_folder, archive_file),
                                  inv_stiff_table, arch_id_rule)
        heatmap_3d(arch, title, out_name, max_fit)
    else:
        _, arch = load_archive(os.path.join(results_folder, archive_file),
                               inv_stiff_table, arch_id_rule)
        heatmap(arch, title, out_name, max_fit)


def main(results_folder, max_fit=None):
    plot_params = {
        'shape': (MAX_NUM_MODULES - MIN_NUM_MODULES+1, len(STIFF_TABLE)),
        'inv_stiff_table': {str(k): v
                            for k, v in zip(STIFF_TABLE, range(len(STIFF_TABLE)))}
    }

    archive_files = [f for f in os.listdir(results_folder)
                     if re.match('archive_gen_[0-9]+\.csv', f)]
    arch_id_rule = re.compile('[0-9]+')
    archives = [load_archive(os.path.join(results_folder, a), plot_params, arch_id_rule)
                for a in archive_files]
    archives.sort(key=lambda k: k[0])

    # note: remove archive ids before passing them to the visualization function
    arch_evolution(list(zip(*archives))[1], fps=15,
                   out_folder=results_folder, max_fit=max_fit)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for plotting the evolution'
                                                 'of the Multi-dimensional Archive of Phenotypic Elites')
    parser.add_argument('results_folder', metavar='results_folder', type=str, nargs='?',
                        default='../../results/map_elites',
                        help='the folder where results archives are stored')
    parser.add_argument('archive_file', metavar='archive_file', type=str, nargs='?',
                        default='archive_213_0.csv',
                        help='the archive file where the features are stored')
    parser.add_argument('--max-fit', metavar='max-fit', type=int, action='store',
                        default=None, help='maximum fitness value provided in the color bar')
    parser.add_argument('--single', dest='is_single', action='store_const',
                        const=True, help='Select whether to produce a single frame.')
    parser.add_argument('--noisy', dest='is_noisy', action='store_const',
                        const=True, help='Select whether results have been produced by a noisy evolution.')
    parser.add_argument('--3d', dest='is_3d', action='store_const',
                        const=True, help='select whether to consider two or three MAP dimensions.')

    args = parser.parse_args()

    if args.is_single:
        single_frame(args.results_folder, args.archive_file, args.max_fit, args.is_noisy, args.is_3d)
    else:
        main(args.results_folder, args.max_fit)
