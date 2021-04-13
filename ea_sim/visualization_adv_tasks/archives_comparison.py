import argparse
import numpy as np
import os
import re
import subprocess
import sys

import matplotlib
# select matplotlib backend
matplotlib.use('pdf')
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from params_conf import N_MODULES, STIFF_TABLE
from visualization_adv_tasks.generate_heatmaps import load_morph_archive, load_contr_archive, \
    load_single_archive, project_to_double


def load_archives(exp_folder, deep_grid, single_map, morph_only, best_function, max_num_archives):
    if not single_map:
        morph_archive_files = [os.path.join(exp_folder, 'archives_morphologies', f)
                               for f in os.listdir(os.path.join(exp_folder, 'archives_morphologies'))
                               if re.match('entity_archive.*\.csv', f)]
        morph_archive_files.sort(key=lambda f: int(re.split('_|\.', f)[-2]))
        if max_num_archives:
            morph_archive_files = morph_archive_files[:max_num_archives]

        if not morph_only:
            contr_archive_files = [os.path.join(exp_folder, 'archives_controllers', f)
                                   for f in os.listdir(os.path.join(exp_folder, 'archives_controllers'))
                                   if re.match('nn_archive.*\.csv', f)]
            contr_archive_files.sort(key=lambda f: int(re.split('_|\.', f)[-2]))

            contr_archive_bounds_files = [os.path.join(exp_folder, 'archives_controllers', f)
                                          for f in os.listdir(os.path.join(exp_folder, 'archives_controllers'))
                                          if re.match('nn_archive_bounds.*\.json', f)]
            contr_archive_bounds_files.sort(key=lambda f: int(re.split('_|\.', f)[-2]))

            if max_num_archives:
                contr_archive_files = contr_archive_files[:max_num_archives]
                contr_archive_bounds_files = contr_archive_bounds_files[:max_num_archives]
        else:
            contr_archive_files, contr_archive_bounds_files = [], []

        print('\tMorphologies archives')
        morph_archives = [load_morph_archive(maf) for maf in morph_archive_files]

        if not morph_only:
            print('\tControllers archives')
            contr_archives_data = [load_contr_archive(caf, cabf, deep_grid, best_function)
                                   for caf, cabf in zip(contr_archive_files, contr_archive_bounds_files)]
            contr_archives, contr_archives_bounds = list(zip(*contr_archives_data))
        else:
            contr_archives, contr_archives_bounds = [], []
    else:
        single_archives_files = [os.path.join(exp_folder, 'single_archives', f)
                                 for f in os.listdir(os.path.join(exp_folder, 'single_archives'))
                                 if re.match('archive.*\.csv', f)]
        single_archives_files.sort(key=lambda f: int(re.split('_|\.', f)[-2]))

        contr_archive_bounds_files = [os.path.join(exp_folder, 'single_archives', f)
                                      for f in os.listdir(os.path.join(exp_folder, 'single_archives'))
                                      if re.match('nn_section_bounds.*\.json', f)]
        contr_archive_bounds_files.sort(key=lambda f: int(re.split('_|\.', f)[-2]))

        if max_num_archives:
            single_archives_files = single_archives_files[:max_num_archives]
            contr_archive_bounds_files = contr_archive_bounds_files[:max_num_archives]

        print('\tSingle archives')
        single_archives = [
            load_single_archive(archive_file, bounds_file)
            for archive_file, bounds_file in zip(single_archives_files, contr_archive_bounds_files)
        ]
        morph_archives, contr_archives, contr_archives_bounds = \
            list(zip(*[(*(project_to_double(archive, bounds, best_function)), bounds)
                       for archive, bounds in single_archives]))

    return morph_archives, contr_archives, contr_archives_bounds


def plot_morph_archives(conf):
    matplotlib.rcParams['pdf.fonttype'] = 42
    matplotlib.rcParams['ps.fonttype'] = 42
    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    fit_values = []

    fig, axes = plt.subplots(1, len(conf['archives']), sharey='all', figsize=(15, 10))
    for ax, arch in zip(axes, conf['archives']):
        fit_values.append([])
        parch = np.full(shape=tuple(reversed(arch.shape)), fill_value=np.NaN)
        for i in range(arch.shape[0]):
            for j in range(arch.shape[1]):
                if arch[i, j] is not None:
                    fit_val = arch[i, j]

                    if conf['fit_diff']:
                        fit_val = conf['init_fit'] - fit_val
                    if conf['invert_fit']:
                        fit_val = 1.0/fit_val

                    parch[j, i] = fit_val
                    fit_values[-1].append(fit_val)

        ax.set_xlabel(conf['x_label'], labelpad=7, fontweight='light', fontsize=16)
        ax.set_xticks(conf['x_ticks'])
        ax.set_xticklabels(conf['x_ticks_labels'])
        im = ax.imshow(parch, cmap='viridis', origin='lower', vmin=conf['min_val'], vmax=conf['max_val'])

    # set y axis information
    axes[0].set_ylabel(conf['y_label'], labelpad=3, fontweight='light', fontsize=16)
    axes[0].set_yticks(conf['y_ticks'])
    axes[0].set_yticklabels(conf['y_ticks_labels'])

    # fix y ticks
    for ax in axes[1:]:
        ax.tick_params(axis='y', which='both', length=0)

    # color bar details
    im_ratio = conf['archives'][-1].shape[1] / conf['archives'][-1].shape[0]
    fig.subplots_adjust(right=0.97)
    cbar_ax = fig.add_axes([0.98, 0.3587, 0.01, im_ratio * 0.245])
    cbar = fig.colorbar(im, cax=cbar_ax)
    cbar.set_label('Fitness', rotation=270, labelpad=20, fontsize=16)

    plt.subplots_adjust(wspace=0.04)
    plt.savefig(conf['out_file'], bbox_inches='tight')
    plt.close()

    print('Morphology archives')
    qds = []
    for i, fits in enumerate(fit_values):
        print('Map #{}, min fit: {}, max fit: {}'.format(i, np.min(fits), np.max(fits)))
        qds.append(np.sum(fits))
    print()
    print('QDS: {}\nmean QD: {}\nstd QD: {}\nmin QD: {}\nmax QD: {}\nmedian QD: {}'.format(
        qds,
        np.mean(qds),
        np.std(qds),
        np.min(qds),
        np.max(qds),
        np.median(qds)
    ))
    print()


def plot_contr_archives(conf):
    matplotlib.rcParams['pdf.fonttype'] = 42
    matplotlib.rcParams['ps.fonttype'] = 42
    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'normal'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    fit_values = []

    fig, axes = plt.subplots(1, len(conf['archives']), sharey='all', figsize=(15, 10))
    for ax, arch, bounds in zip(axes, conf['archives'], conf['bounds']):
        fit_values.append([])
        parch = np.full(shape=tuple(reversed(arch.shape)), fill_value=np.NaN)
        for i in range(arch.shape[0]):
            for j in range(arch.shape[1]):
                if arch[i, j] is not None:
                    fit_val = arch[i, j]

                    if conf['fit_diff']:
                        fit_val = conf['init_fit'] - fit_val
                    if conf['invert_fit']:
                        fit_val = 1.0 / fit_val

                    parch[j, i] = fit_val
                    fit_values[-1].append(fit_val)

        ax.set_xlabel('1st dimension', labelpad=7, fontweight='light', fontsize=16)
        ax.set_xticks(range(arch.shape[0]))
        ax.set_xticklabels([i for i in range(arch.shape[0])])

        im = ax.imshow(parch, cmap='viridis', origin='lower', vmin=conf['min_val'], vmax=conf['max_val'])

    axes[0].set_ylabel('2nd dimension', labelpad=3, fontweight='light', fontsize=16)
    axes[0].set_yticks(range(conf['archives'][-1].shape[1]))
    axes[0].set_yticklabels([i for i in range(conf['archives'][-1].shape[1])])

    # fix y ticks
    for ax in axes[1:]:
        ax.tick_params(axis='y', which='both', length=0)

    # color bar details
    im_ratio = conf['archives'][-1].shape[1] / conf['archives'][-1].shape[0]
    fig.subplots_adjust(right=0.97)
    cbar_ax = fig.add_axes([0.98, 0.3587, 0.01, im_ratio * 0.245])
    cbar = fig.colorbar(im, cax=cbar_ax)
    cbar.set_label('Fitness', rotation=270, labelpad=20, fontsize=16)

    plt.subplots_adjust(wspace=0.04)
    plt.savefig(conf['out_file'], bbox_inches='tight')
    plt.close()

    print('Controller archives')
    qds = []
    for i, fits in enumerate(fit_values):
        print('Map #{}, min fit: {}, max fit: {}'.format(i, np.min(fits), np.max(fits)))
        qds.append(np.sum(fits))
    print()
    print('QDS: {}\nmean QD: {}\nstd QD: {}\nmin QD: {}\nmax QD: {}\nmedian QD: {}'.format(
        qds,
        np.mean(qds),
        np.std(qds),
        np.min(qds),
        np.max(qds),
        np.median(qds)
    ))


def main(exp_folder, deep_grid, single_map, morph_only, func_best, fit_diff, init_fit, invert_fit, min_vals, max_vals,
         max_num_archives, parent_folders_num, out_files, owner):
    if func_best not in ['min', 'max']:
        raise Exception('The function provided to determine the best fitness'
                        + 'is not valid')
    else:
        function_best = min if func_best == 'min' else max

    if len(min_vals) != 2:
        raise Exception('The number of minimum values for the color bars provided '
                        + 'is not valid')

    if len(max_vals) != 2:
        raise Exception('The number of maximum values for the color bars provided '
                        + 'is not valid')

    print('Loading archives...')
    morph_archives, contr_archives, contr_archives_bounds = \
        load_archives(exp_folder, deep_grid, single_map, morph_only, function_best, max_num_archives)

    parent_folder = os.path.normpath(exp_folder)
    for i in range(0, parent_folders_num):
        parent_folder = os.path.dirname(parent_folder)
    out_folder = os.path.join(parent_folder, 'evo_plots')
    os.makedirs(out_folder, exist_ok=True)

    print('Plotting results...\n')
    morph_archives_conf = {
        'archives': morph_archives,
        'x_ticks': range(len(N_MODULES)),
        'x_ticks_labels': N_MODULES,
        'y_ticks': range(len(STIFF_TABLE)),
        'y_ticks_labels': STIFF_TABLE,
        'x_label': '# modules',
        'y_label': 'Stiffness',
        'min_val': min_vals[0],
        'max_val': max_vals[0],
        'fit_diff': fit_diff,
        'init_fit': init_fit,
        'invert_fit': invert_fit,
        'out_file': os.path.join(out_folder, out_files[0])
    }
    plot_morph_archives(morph_archives_conf)

    if not morph_only:
        contr_archives_conf = {
            'archives': contr_archives,
            'bounds': contr_archives_bounds,
            'min_val': min_vals[1],
            'max_val': max_vals[1],
            'fit_diff': fit_diff,
            'init_fit': init_fit,
            'invert_fit': invert_fit,
            'out_file': os.path.join(out_folder, out_files[1])
        }
        plot_contr_archives(contr_archives_conf)

    if owner is not None and len(owner) == 2:
        try:
            exec_string = 'chown -R {}:{} {}'.format(owner[0], owner[1], out_folder)
            c_proc = subprocess.run(exec_string.split(' '), capture_output=True)
        except:
            raise Exception('An error occurred during the owner setting')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for plotting archives (morphology and controller) of '
                                                 'multiple experiments.')
    parser.add_argument('exp_folder', metavar='exps_folder', type=str, nargs='?',
                        help='experiment folder containing "archives_controllers" and '
                             '"archives_morphologies" directories (each of them contains the related .csv files)')
    parser.add_argument('--deep-grid', dest='deep_grid', action='store_const',
                        const=True, default=False, help='is this a deep grid execution?')
    parser.add_argument('--single-map', dest='single_map', action='store_const',
                        const=True, default=False, help='is this a single map execution?')
    parser.add_argument('--morph-only', dest='morph_only', action='store_const',
                        const=True, default=False, help='is only the morphology map present?')
    parser.add_argument('--fit-difference', dest='fit_difference', action='store_const',
                        const=True, default=False, help='show fitness values as (init_fit - fit_value)')
    parser.add_argument('--init-fit', metavar='init-fit', type=float, action='store',
                        default=45, help='initial fit value, used in combination with fit-difference')
    parser.add_argument('--invert-fit', dest='invert_fit', action='store_const',
                        const=True, default=False, help='invert fitness values')
    parser.add_argument('--fit-func-best', metavar='fitness_function_best', type=str, nargs='?', default='min',
                        help='function used to determine the best fitness, allowed values are: min, max')
    parser.add_argument('--min-vals', metavar='min-vals', type=float, action='store', nargs='+',
                        default=[0, 0], help='minimum values provided in the color bars (must be 2)')
    parser.add_argument('--max-vals', metavar='max-vals', type=float, action='store', nargs='+',
                        default=[50, 100], help='maximum values provided in the color bars (must be 2)')
    parser.add_argument('--max-num-archives', metavar='max-num-archives', type=int, action='store',
                        default=None, help='maximum number of archives to load')
    parser.add_argument('--parent-folders-num', metavar='parent-folders-num', type=int, action='store',
                        default=1, help='number of parent folders to traverse before creating the out folder')
    parser.add_argument('--out-files', metavar='out-files', type=str, action='store', nargs='+',
                        default=['archives_morph.pdf', 'archives_contr.pdf'],
                        help='output filenames (must be 2)')
    parser.add_argument('--owner', metavar='owner', type=str, action='store', nargs='+',
                        default=None, help='User and group for chown')

    args = parser.parse_args()
    main(args.exp_folder, args.deep_grid, args.single_map, args.morph_only, args.fit_func_best,
         args.fit_difference, args.init_fit, args.invert_fit, args.min_vals, args.max_vals,
         args.max_num_archives, args.parent_folders_num, args.out_files, args.owner)
