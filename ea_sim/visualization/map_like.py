#! /usr/bin/env python3

import os
import sys
import argparse

import matplotlib
import numpy as np
# select matplotlib backend
matplotlib.use('pdf')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from params_conf import MIN_NUM_MODULES, MAX_NUM_MODULES, N_MODULES, STIFF_TABLE

# ================================ #
#           PREPROCESSING          #
# ================================ #
def extract_features(robot_string):
    modules = robot_string.split('--')[:-1]
    # we consider only the first module since
    # all the modules have the same stiffness
    properties = modules[0].split('-')

    # last property of each module is its stiffness
    return len(modules), float(properties[-1])

def extract_features_3d(robot_string):
    modules = robot_string.split('--')[:-1]
    # we consider only the first module since
    # all the modules have the same stiffness
    properties = [mod.split('-') for mod in modules]
    stiff_vals = [float(prop[-1]) for prop in properties]

    # last property of each module is its stiffness
    return len(modules), min(stiff_vals), max(stiff_vals)

def read_history_file(history_file, is_3d):
    history_data = {}
    with open(history_file) as history_in:
        # skip header
        history_in.readline()
        for line in history_in:
            robot_string, fitness = line.strip().split(',')
            if is_3d:
                features = extract_features_3d(robot_string)
            else:
                features = extract_features(robot_string)
            fitness = float(fitness)

            # keep in the history only the best individual for that configuration
            if features in history_data:
                if history_data[features] < fitness:
                    history_data[features] = fitness
            else:
                history_data[features] = fitness

    return history_data

# ================================ #
#           VISUALIZATION          #
# ================================ #
def heatmap(parch, title, out_name, max_fit=None):
    font = {'family': 'Source Sans Pro', 'size': 16, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    fig = plt.figure(figsize=(12, 5.5))
    ax = fig.gca()
    ax.set_title(title, size=24, fontweight='normal')
    ax.set_xlabel('Stiffness value', labelpad=10, fontweight='light')
    ax.set_ylabel('# modules', labelpad=10, fontweight='light')

    im = plt.imshow(parch, cmap='viridis', origin='lower')

    im_ratio = parch.shape[0] / parch.shape[1]
    cbar = fig.colorbar(im, ax=ax, fraction=im_ratio * 0.047, pad=0.04)
    cbar.set_label('Fitness', rotation=270, labelpad=20, fontweight='light')
    if max_fit is not None:
        plt.clim(5, max_fit)

    if STIFF_TABLE is not None:
        plt.xticks(range(len(STIFF_TABLE)), STIFF_TABLE)
    if MAX_NUM_MODULES is not None and MIN_NUM_MODULES is not None:
        plt.yticks(range(len(N_MODULES)), N_MODULES)
    plt.savefig(out_name, bbox_inches='tight')

def heatmap_3d(arch, title, out_name, max_fit=None):
    font = {'family': 'Source Sans Pro', 'size': 16, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    d1 = len(N_MODULES)
    d2 = d3 = len(STIFF_TABLE)

    fig = plt.figure(figsize=(15, 7))
    ax = fig.gca()
    ax.set_title(title, size=24, fontweight='normal')
    ax.set_xlabel('Min Module Stiffness', labelpad=10)
    ax.set_ylabel('Max Module Stiffness', labelpad=10)

    # Major ticks
    ax.set_xticks(np.arange(-.5, d1*d2, d1))
    ax.set_yticks(np.arange(0, d3, 1))

    # prepare custom ticks labels
    m_labels = []
    for st in STIFF_TABLE:
        for k in range(d1//2):
            m_labels.append('')
        m_labels.append(st)
        for k in range(d1//2):
            m_labels.append('')

    # Labels for major ticks
    ax.set_xticklabels(m_labels, minor=True)
    ax.set_yticklabels(STIFF_TABLE)

    # Minor ticks
    ax.set_xticks(np.arange(-.5, d1*d2, 1), minor=True)
    ax.set_yticks(np.arange(-.5, d3, 1), minor=True)

    ax_top = ax.twiny()
    ax_top.set_xticks(np.arange(0, d1 * d2, 1), minor=True)
    ax_top.set_xticklabels(N_MODULES * d2, minor=True, fontsize=5, rotation=90)
    ax_top.set_xlabel('# Modules', labelpad=10)

    # fix zorder of first x axis
    ax.set_zorder(10)
    ax.patch.set_visible(False)

    # Gridlines based on minor ticks
    ax.grid(axis='x', which='minor', color='lightgray', linestyle='-', linewidth=0.25, alpha=0.4)
    ax.grid(axis='x', which='major', color='k', linestyle='-', linewidth=0.75)
    ax.grid(axis='y', which='minor', color='k', linestyle='-', linewidth=1)
    ax.tick_params(axis='x', which='minor', bottom=False, top=False, labelbottom=True, labeltop=False)
    ax.tick_params(axis='x', which='major', bottom=False, top=False, labelbottom=False)
    ax.tick_params(axis='y', which='both', left=False, right=False, labelleft=True)

    ax_top.tick_params(axis='x', which='minor', bottom=False, top=False, labelbottom=False, labeltop=True)
    ax_top.tick_params(axis='x', which='major', bottom=False, top=False, labeltop=False)

    im = plt.imshow(arch, cmap='viridis', aspect='auto', origin='lower')

    im_ratio = arch.shape[0] / arch.shape[1]
    cbar = fig.colorbar(im, ax=ax, fraction=im_ratio, pad=0.04)
    cbar.set_label('Fitness', rotation=270, labelpad=20)
    if max_fit is not None:
        plt.clim(5, max_fit)
    plt.savefig(out_name, bbox_inches='tight')
    plt.close()

# ================================ #
#              PROGRAM             #
# ================================ #
def plot(wrk_folder, h_file, max_fit=None, seed=None, sim_id=None, is_3d=False):
    history = read_history_file(os.path.join(wrk_folder, h_file), is_3d)

    out_id = ' '.join(os.path.splitext(h_file)[0].split('_')[:1])
    out_name = os.path.join(wrk_folder,
                            'archive_{}{}_{}_{}.pdf'.format(out_id,
                                                            '_3d' if is_3d else '',
                                                            seed,
                                                            sim_id))

    # try to extract experiment algorithm name
    title_comps = os.path.basename(os.path.normpath(wrk_folder)).split('_')
    if len(title_comps) > 2:
        title = '{} Features Space - {}'.format(title_comps[1], title_comps[-1])
    else:
        title = 'Features Space'

    if is_3d:
        d1 = len(N_MODULES)
        d2 = d3 = len(STIFF_TABLE)
        arch = np.full(shape=(d3, d1 * d2), fill_value=np.NaN)
        for prop, fit in history.items():
            second_idx = STIFF_TABLE.index(prop[1]) * d1 + (prop[0] - MIN_NUM_MODULES)
            arch[STIFF_TABLE.index(prop[2]), second_idx] = fit
        heatmap_3d(arch, title, out_name, max_fit)
    else:
        parch = np.full(shape=(len(N_MODULES), len(STIFF_TABLE)), fill_value=np.NaN)
        for prop, fit in history.items():
            parch[(prop[0] - MIN_NUM_MODULES), STIFF_TABLE.index(prop[1])] = fit
        heatmap(parch, title, out_name, max_fit)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for plotting the MAP-Elites like archive')
    parser.add_argument('wrk_folder', metavar='wrk_folder', type=str, nargs='?',
                        default='../../results/mu+lambda',
                        help='working directory where history is stored')
    parser.add_argument('history_file', metavar='history_file', type=str, nargs='?',
                        default='history_213_sim_0.csv',
                        help='history file')
    parser.add_argument('--max-fit', metavar='max-fit', type=int, action='store',
                        default=None, help='maximum fitness value provided in the color bar')
    parser.add_argument('--3d', dest='is_3d', action='store_const',
                        const=True, help='select whether to consider two or three MAP dimensions.')

    args = parser.parse_args()
    plot(args.wrk_folder, args.history_file, args.max_fit, is_3d=args.is_3d)
