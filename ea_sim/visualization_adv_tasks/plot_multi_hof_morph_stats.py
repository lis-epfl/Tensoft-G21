import argparse
import json
import math
import numpy as np
import os
import subprocess
import re
import sys

import matplotlib
# select matplotlib backend
matplotlib.use('pdf')
import matplotlib.pyplot as plt
from scipy.stats import ranksums

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from params_conf import MIN_NUM_MODULES, N_MODULES, STIFF_TABLE
from utils import parse_robot_string
from visualization_adv_tasks.archives_comparison import plot_morph_archives
from visualization.plot_actuation_stats import plot_stats_boxplot


def load_hof_stats(i, hall_of_fame_file, morphologies_file, max_num_individuals, best_function,
                   fit_diff, init_fit, invert_fit):
    print('\t\tRun #{}'.format(i))

    with open(hall_of_fame_file) as hof_file:
        hall_of_fame = json.load(hof_file)

    with open(morphologies_file) as morph_file:
        morphologies = json.load(morph_file)
    inverse_morph_dict = {v: k for k, v in morphologies.items()}

    counter = 0
    fitnesses = []
    num_modules_dict = {nm: 0 for nm in [i for i in range(2, 11)]}
    stiffness_dict = {sv: 0 for sv in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]}
    heatmap = np.full(shape=(len(N_MODULES), len(STIFF_TABLE)), fill_value=None)

    for fitness, pair_list in hall_of_fame.items():
        fit_val = float(fitness)
        if fit_diff:
            fit_val = init_fit - fit_val
        if invert_fit:
            fit_val = 1.0 / fit_val

        for m_id, c_id in pair_list:
            robot = parse_robot_string(inverse_morph_dict[m_id])

            fitnesses.append(fit_val)
            num_modules_dict[len(robot)] += 1
            stiffness_dict[robot[0]['stiff']] += 1

            index_1 = len(robot) - MIN_NUM_MODULES
            index_2 = STIFF_TABLE.index(robot[0]['stiff'])

            if heatmap[index_1, index_2] is not None:
                heatmap[index_1, index_2] = best_function(heatmap[index_1, index_2], float(fitness))
            else:
                heatmap[index_1, index_2] = float(fitness)

            counter += 1

            if counter == max_num_individuals:
                break
        if counter == max_num_individuals:
            break

    return fitnesses, num_modules_dict, stiffness_dict, heatmap


def load_experiment(i, folder, max_num_individuals, best_function, fit_diff, init_fit, invert_fit):
    print('\tExperiment #{}'.format(i))

    # load single run hall of fame stats
    hall_of_fames = [
        (
            os.path.join(folder, f),
            os.path.join(folder, f.replace('.json', ''), 'morphologies_{}.json'.format(f.split('_')[-2]))
        )
        for f in os.listdir(folder) if re.match('hall_of_fame.*\.json', f)
    ]
    hall_of_fames.sort(key=lambda element: int(re.split('_|\.', element[0])[-2]))

    fitness_vals_lists, num_modules_dicts, stiffness_dicts, heatmaps = \
        list(zip(*[load_hof_stats(j, hof_f, morph_f, max_num_individuals, best_function, fit_diff, init_fit, invert_fit)
                   for j, (hof_f, morph_f) in enumerate(hall_of_fames)]))

    fitness_vals = [fit_val for fit_vals in fitness_vals_lists for fit_val in fit_vals]
    fitness_avg_vals = [np.mean(fit_vals) for fit_vals in fitness_vals_lists]

    num_modules = {nm: [] for nm in [i for i in range(2, 11)]}
    for key in num_modules.keys():
        for num_modules_dict in num_modules_dicts:
            num_modules[key].append(num_modules_dict[key])
        num_modules[key] = (np.mean(num_modules[key]), np.std(num_modules[key]))

    stiffness = {sv: [] for sv in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]}
    for key in stiffness.keys():
        for stiffness_dict in stiffness_dicts:
            stiffness[key].append(stiffness_dict[key])
        stiffness[key] = (np.mean(stiffness[key]), np.std(stiffness[key]))

    return fitness_vals, fitness_avg_vals, num_modules, stiffness, list(heatmaps)


def plot_barplots(conf):
    font = {'family': 'Source Sans Pro', 'size': 13, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    colors = plt.cm.viridis(np.linspace(0, 1, len(conf['data'][0].keys())))

    if len(conf['data']) == 1:
        fig = plt.figure(figsize=(12, 8))
        ax = fig.gca()

        x, y, std = list(zip(*[(str(k), v[0], v[1])
                               for k, v in sorted(conf['data'][0].items(), key=lambda r: float(r[0]))]))
        bar_plot = ax.bar(x, y, yerr=(std if not conf['ratio'] else None),
                          error_kw=dict(ecolor='gray', lw=0.5, capsize=3, capthick=0.5), color=colors)

        for j, rect in enumerate(bar_plot):
            height = rect.get_height()
            ax.text(rect.get_x() + rect.get_width() / 2., height + (0.01 if conf['ratio'] else 0.3),
                    round(y[j], 2), ha='center', va='bottom', rotation=0)

        ax.set_xlabel(conf['x_labels'][0], labelpad=7, fontweight='light', fontsize=13)
        ax.set_ylim(0, 1 if conf['ratio'] else ((math.ceil(conf['y_max'] / 10) + 1) * 10))
        ax.set_ylabel('Avg. # individuals', labelpad=3, fontweight='light', fontsize=13)
    else:
        fig, axes = plt.subplots(1, len(conf['data']), sharey='all', figsize=(4*len(conf['data']), 4))
        fig.suptitle(conf['title'], y=0.95, fontweight='normal', fontsize=18)

        for ax, (i, d) in zip(axes, enumerate(conf['data'])):
            x, y, std = list(zip(*[(str(k), v[0], v[1])
                                   for k, v in sorted(d.items(), key=lambda r: float(r[0]))]))
            bar_plot = ax.bar(x, y, yerr=(std if not conf['ratio'] else None),
                              error_kw=dict(ecolor='gray', lw=0.5, capsize=3, capthick=0.5), color=colors)

            for j, rect in enumerate(bar_plot):
                height = rect.get_height()
                ax.text(rect.get_x() + rect.get_width() / 2., height + (0.01 if conf['ratio'] else 0.3),
                        round(y[j], 2), ha='center', va='bottom', rotation=0)

            ax.set_xlabel(conf['x_labels'][i], labelpad=7, fontweight='light', fontsize=13)
            ax.set_ylim(0, 1 if conf['ratio'] else ((math.ceil(conf['y_max'] / 10) + 1) * 10))

        axes[0].set_ylabel('Avg. # individuals', labelpad=3, fontweight='light', fontsize=13)

        # fix y ticks
        for ax in axes[1:]:
            ax.tick_params(axis='y', which='both', length=0)

        plt.subplots_adjust(wspace=0.04)

    plt.savefig(conf['out_file'], bbox_inches='tight')
    plt.close()


def main(exps_folders, max_num_individuals, best_function, fit_diff, init_fit, invert_fit, min_val, max_val,
         parent_folders_num, out_files, labels, owner):
    if best_function not in ['min', 'max']:
        raise Exception('The function provided to determine the best fitness'
                        + 'is not valid')
    else:
        function_best = min if best_function == 'min' else max

    print('Loading experiments...')
    fitness_vals, fitness_avg_vals, num_modules, stiffness, exps_heatmaps =\
        list(zip(*[load_experiment(i + 1, f, max_num_individuals, function_best, fit_diff, init_fit, invert_fit)
                   for i, f in enumerate(exps_folders)]))

    if len(out_files) != 4:
        raise Exception('The number of provided output filenames is not valid, 4 are required!')

    if labels is not None:
        if len(labels) != len(exps_folders):
            raise Exception('The number of provided labels'
                            + 'is different from the number'
                            + f'of experiments: {len(labels)}!={len(exps_folders)}')
    else:
        labels = [os.path.dirname(os.path.dirname(exp_folder)).rsplit('/')[-1] for exp_folder in exps_folders]

    parent_folder = os.path.normpath(exps_folders[0])
    for i in range(0, parent_folders_num):
        parent_folder = os.path.dirname(parent_folder)
    out_folder = os.path.join(parent_folder, 'evo_plots')
    os.makedirs(out_folder, exist_ok=True)

    for fit_vals, label in zip(fitness_vals, labels):
        print('{}\n\tfit vals: {}\n\tavg: {}\n\tstd: {}\n\tmin: {}\n\tmax {}\n\tmed: {}\n'.format(
            label,
            fit_vals,
            np.mean(fit_vals),
            np.std(fit_vals),
            np.min(fit_vals),
            np.max(fit_vals),
            np.median(fit_vals)
        ))

    for i in range(0, len(fitness_vals)):
        for j in range(i+1, len(fitness_vals)):
            print('{} - {}\n\t{}'.format(
                labels[i],
                labels[j],
                ranksums(fitness_vals[i], fitness_vals[j]))
            )

    print('Plotting results...')
    fitness_boxplot_conf = {
        'data': list(fitness_vals),
        'title': 'Distribution across runs of best overall fitness',
        'x_tick_labels': labels,
        'y_label': 'Fitness [cm]',
        'ratio': False,
        'out_file': os.path.join(out_folder, out_files[0])
    }
    plot_stats_boxplot(fitness_boxplot_conf)

    '''
    contr_evo_best_fits = [
        [29.468108125, 28.084806874999998, 29.222787499999995, 24.83102875, 25.646933125],
        [23.476965, 35.31125, 29.381075624999998, 27.094949375, 27.262569375]
    ]
    best_fits_comp = [contr_evo_best_fits[0], list(fitness_vals)[0]]

    best_fits_boxplots_conf = {
        'data': best_fits_comp,
        'title': 'Distribution across runs of best overall fitness',
        'x_tick_labels': ['Controller Evolution\nExp 1', 'Co-Evolution\n' + str(labels[0])],
        'y_label': 'Fitness [cm]',
        'ratio': False,
        'out_file': os.path.join(out_folder, 'contr_evo_coev_fit_comp.pdf')
    }
    plot_stats_boxplot(best_fits_boxplots_conf)
    print(ranksums(best_fits_comp[0], best_fits_comp[1]))
    '''

    fitness_avg_boxplot_conf = {
        'data': list(fitness_avg_vals),
        'title': 'Distribution across runs of average fitness in Hall of Fame' +
                 (' (size={})'.format(max_num_individuals) if max_num_individuals != 101 else ''),
        'x_tick_labels': labels,
        'y_label': 'Avg. Fitness [cm]',
        'ratio': False,
        'out_file': os.path.join(out_folder, out_files[0].replace('.pdf', '_avg.pdf'))
    }
    plot_stats_boxplot(fitness_avg_boxplot_conf)

    num_modules_barplots_conf = {
        'data': list(num_modules),
        'title': 'Average distribution across runs of number of modules in Hall of Fame' +
                 (' (size={})'.format(max_num_individuals) if max_num_individuals != 101 else ''),
        'x_labels': [('{} - '.format(label) if len(labels) > 1 else '')+'# modules'
                     for label in labels],
        'ratio': max_num_individuals == 1,
        'y_max': max([x+std for nm_dict in num_modules for (x, std) in nm_dict.values()]),
        'out_file':  os.path.join(
            out_folder,
            out_files[1]
        )
    }
    plot_barplots(num_modules_barplots_conf)

    stiffness_barplots_conf = {
        'data': list(stiffness),
        'title': 'Average distribution across runs of stiffness in Hall of Fame' +
                 (' (size={})'.format(max_num_individuals) if max_num_individuals != 101 else ''),
        'x_labels': [('{} - stiffness'.format(label) if len(labels) > 1 else 'Stiffness')
                     for label in labels],
        'ratio': max_num_individuals == 1,
        'y_max': max([x+std for st_dict in stiffness for (x, std) in st_dict.values()]),
        'out_file': os.path.join(
            out_folder,
            out_files[2]
        )
    }
    plot_barplots(stiffness_barplots_conf)

    for heatmaps, label in zip(exps_heatmaps, labels):
        heatmaps_conf = {
            'archives': heatmaps,
            'x_ticks': range(len(N_MODULES)),
            'x_ticks_labels': N_MODULES,
            'y_ticks': range(len(STIFF_TABLE)),
            'y_ticks_labels': STIFF_TABLE,
            'x_label': '# modules',
            'y_label': 'Stiffness',
            'fit_diff': fit_diff,
            'init_fit': init_fit,
            'invert_fit': invert_fit,
            'min_val': min_val,
            'max_val': max_val,
            'out_file': os.path.join(out_folder,
                                     out_files[3].replace('.pdf', '_{}.pdf'.format(label.replace(' ', '_').lower())))
        }
        plot_morph_archives(heatmaps_conf)

    if owner is not None and len(owner) == 2:
        try:
            exec_string = 'chown -R {}:{} {}'.format(owner[0], owner[1], out_folder)
            c_proc = subprocess.run(exec_string.split(' '), capture_output=True)
        except:
            raise Exception('An error occurred during the owner setting')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for plotting average statistics (mainly morphological) about '
                                                 'the individuals contained in the hall of fame of multiple experiments'
                                                 ' (fitness, # of modules and stiffness distributions) and '
                                                 'MAP-Elites like heatmaps.')
    parser.add_argument('exps_folders', metavar='exps_folders', type=str, nargs='+',
                        help='list of experiments folders containing hall_of_fame files and corresponding directories '
                             'with morphologies list')
    parser.add_argument('--parent-folders-num', metavar='parent-folders-num', type=int, action='store',
                        default=1, help='number of parent folders to traverse before creating the out folder')
    parser.add_argument('--max-num-individuals', metavar='max-num-individuals', type=int, action='store',
                        default=101, help='maximum number of individuals to consider in Hall of Fames')
    parser.add_argument('--fit-func-best', metavar='fitness_function_best', type=str, nargs='?', default='min',
                        help='function used to determine the best fitness, allowed values are: min, max')
    parser.add_argument('--fit-difference', dest='fit_difference', action='store_const',
                        const=True, default=False, help='show fitness values as (init_fit - fit_value)')
    parser.add_argument('--init-fit', metavar='init-fit', type=float, action='store',
                        default=45, help='initial fit value, used in combination with fit-difference')
    parser.add_argument('--invert-fit', dest='invert_fit', action='store_const',
                        const=True, default=False, help='invert fitness values')
    parser.add_argument('--min-val', metavar='min-val', type=float, action='store',
                        default=0, help='minimum value provided in the color bar')
    parser.add_argument('--max-val', metavar='max-val', type=float, action='store',
                        default=45, help='maximum value provided in the color bar')
    parser.add_argument('--out-files', metavar='out-files', type=str, action='store', nargs='+',
                        default=['hof_fitness.pdf', 'hof_num_modules.pdf', 'hof_stiffness.pdf', 'hof_heatmaps.pdf'],
                        help='output filenames (must be 4)')
    parser.add_argument('--labels', metavar='labels', type=str, action='store', nargs='+',
                        default=None, help='charts labels - must match the number of experiments!')
    parser.add_argument('--owner', metavar='owner', type=str, action='store', nargs='+',
                        default=None, help='User and group for chown')

    args = parser.parse_args()
    main(args.exps_folders, args.max_num_individuals, args.fit_func_best, args.fit_difference, args.init_fit,
         args.invert_fit, args.min_val, args.max_val, args.parent_folders_num, args.out_files, args.labels, args.owner)
