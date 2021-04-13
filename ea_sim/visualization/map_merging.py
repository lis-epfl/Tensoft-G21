#! /usr/bin/env python3

import os
import sys
import argparse

import matplotlib
import numpy as np
# select matplotlib backend
matplotlib.use('pdf')
import matplotlib.pyplot as plt

sys.path.append('..')
from params_conf import MIN_NUM_MODULES, MAX_NUM_MODULES, N_MODULES, STIFF_TABLE


def extract_features(robot_string):
    modules = robot_string.split('--')[:-1]
    # we consider only the first module since
    # all the modules have the same stiffness
    properties = modules[0].split('-')

    # last property of each module is its stiffness
    return len(modules), float(properties[-1])


def heatmap(archive, title, filename, max_fit=None):
    font = {'family': 'Source Sans Pro', 'size': 16, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    parch = np.full(shape=archive.shape, fill_value=np.NaN)
    for i in range(archive.shape[0]):
        for j in range(archive.shape[1]):
            if archive[i, j] is not None:
                parch[i, j] = archive[i, j]

    fig = plt.figure(figsize=(12, 5.5))
    ax = fig.gca()
    ax.set_title(title, size=26, fontweight='normal')
    ax.set_xlabel('Stiffness value', labelpad=10, fontweight='light')
    ax.set_ylabel('# modules', labelpad=10, fontweight='light')

    im = plt.imshow(parch, cmap='viridis', origin='lower')

    im_ratio = parch.shape[0] / parch.shape[1]
    cbar = fig.colorbar(im, ax=ax, fraction=im_ratio * 0.047, pad=0.04)
    cbar.set_label('Fitness', rotation=270, labelpad=20)
    if max_fit is not None:
        plt.clim(5, max_fit)

    if STIFF_TABLE is not None:
        plt.xticks(range(len(STIFF_TABLE)), STIFF_TABLE)
    if MAX_NUM_MODULES is not None and MIN_NUM_MODULES is not None:
        plt.yticks(range(len(N_MODULES)), N_MODULES)
    plt.savefig(filename, bbox_inches='tight')


def main(input_files, out_file, is_history, max_fit=None, average_same=False, title='ViE Feature Space'):

    if is_history:
        merged_history = {}
        if out_file is None:
            out_file = os.path.join(os.path.dirname(input_files[0]), '_'.join(title.split()) + '.pdf')

        for hfile in input_files:
            with open(hfile) as in_file:
                _ = in_file.readline()  # skip header
                i = 0
                for line in in_file:
                    robot_string, fitness = line.strip().split(',')
                    fitness = float(fitness)
                    if not robot_string in merged_history:
                        merged_history[robot_string] = [fitness]
                    else:
                        merged_history[robot_string].append(fitness)
                    i += 1
                print(hfile, 'history size: {}'.format(i))

        arch_like_map = {}
        for k, v in merged_history.items():
            if average_same:
                # average the fitness value of the same strings obtained by different runs
                merged_history[k] = np.mean(v)
            else:
                merged_history[k] = np.max(v)
            features = extract_features(k)

            if features in arch_like_map:
                if arch_like_map[features] < merged_history[k]:
                    arch_like_map[features] = merged_history[k]
            else:
                arch_like_map[features] = merged_history[k]

        print('final length: {}'.format(len(merged_history)))

        os.makedirs(os.path.dirname(out_file), exist_ok=True)
        with open(os.path.join(os.path.dirname(out_file), 'merged_history.csv'), 'w') as of:
            of.write('rob_string,fitness\n')
            for rob_string, fit in merged_history.items():
                of.write('{},{}\n'.format(rob_string, fit))

        parch = np.full(shape=(len(N_MODULES), len(STIFF_TABLE)), fill_value=np.NaN)
        for prop, fit in arch_like_map.items():
            parch[(prop[0] - MIN_NUM_MODULES), STIFF_TABLE.index(prop[1])] = fit
        heatmap(parch, title, out_file, max_fit)

    else:
        merged_archive = {}
        for archive in input_files:
            with open(archive) as in_file:
                _ = in_file.readline()  # skip header
                i = 0
                for line in in_file:
                    num_mod, stiff, fitness = line.strip().split(',')
                    fitness = float(fitness)
                    aid = (int(num_mod), float(stiff))

                    if not aid in merged_archive:
                        merged_archive[aid] = [fitness]
                    else:
                        merged_archive[aid].append(fitness)
                    i += 1

        parch = np.full(shape=(len(N_MODULES), len(STIFF_TABLE)), fill_value=np.NaN)
        for prop, fit in merged_archive.items():
            parch[(prop[0] - MIN_NUM_MODULES), STIFF_TABLE.index(prop[1])] = np.mean(fit)
        if out_file is None:
            out_file = os.path.join(os.path.dirname(input_files[0]),
                                    'map-elites_feature_map.pdf')

        heatmap(parch, 'MAP-Elites Feature Space', out_file, max_fit)

        arch_filename = os.path.join(os.path.dirname(input_files[0]), 'merged_archive.txt')
        with open(arch_filename, 'w') as arch_file:
            arch_file.write('num_modules,stiffness,fitness\n')
            for k, v in merged_archive.items():
                arch_file.write(f'{k[0]},{k[1]},{np.mean(v)}\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Utility script for merging history files')
    parser.add_argument('history_files', metavar='history_files', type=str, nargs='+',
                        help='list of history files to be merged')
    parser.add_argument('--out-file', metavar='out-file', type=str, action='store',
                        default=None, help='output filename where merged history is stored')
    parser.add_argument('--max-fit', metavar='max-fit', type=float, action='store',
                        default=None, help='maximum fitness value provided in the color bar')
    parser.add_argument('--history', dest='is_history', action='store_const',
                        const=True, help='run the procedure for history files')
    parser.add_argument('--title', metavar='title', type=str, action='store', nargs='?',
                        default='ViE Feature Space', help='Chart title')
    args = parser.parse_args()

    main(args.history_files, args.out_file, args.is_history,
         max_fit=args.max_fit, title=args.title)
