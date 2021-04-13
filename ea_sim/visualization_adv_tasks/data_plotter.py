#! /usr/bin/env python3

import copy
import os
import re
import json
import argparse
import math
import numpy as np
import subprocess

import matplotlib
# select matplotlib backend
matplotlib.use('pdf')
import matplotlib.pyplot as plt

def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def load_evo_file(i, file, chunked, best, worst, limit, fit_diff, init_fit, invert_fit):
    print('\t\tRun #{}'.format(i))

    if not chunked:
        with open(file) as evo_file:
            raw_data = json.load(evo_file)

        single_evo_data = []
        for i, pop in enumerate(raw_data):
            fit_stats = np.asarray([pop[best], pop['avg_fitness'], pop[worst]])
            if fit_diff:
                fit_stats = init_fit - fit_stats
            if invert_fit:
                fit_stats = 1.0/fit_stats

            single_evo_data.append({
                'num_sims': pop['num_sims'],
                'best': fit_stats[0],
                'mean': fit_stats[1],
                'worst': fit_stats[2]
            })
    else:
        single_evo_data = []
        for file_chunk in file:
            print('\t\t\tChunk: {}'.format(file_chunk))
            with open(file_chunk) as chunk_file:
                raw_data_chunk = json.load(chunk_file)

                for i, pop in enumerate(raw_data_chunk):
                    fit_stats = np.asarray([pop[best], pop['avg_fitness'], pop[worst]])
                    if fit_diff:
                        fit_stats = init_fit - fit_stats
                    if invert_fit:
                        fit_stats = 1.0 / fit_stats

                    single_evo_data.append({
                        'num_sims': pop['num_sims'],
                        'best': fit_stats[0],
                        'mean': fit_stats[1],
                        'worst': fit_stats[2]
                    })

                raw_data_chunk = []
                chunk_file.close()
        single_evo_data.sort(key=lambda x: x['num_sims'])

    single_evo_data.insert(0, copy.deepcopy(single_evo_data[0]))
    single_evo_data[0]['num_sims'] = 0

    final_evo_data = {0: {
        'best': single_evo_data[0]['best'],
        'mean': single_evo_data[0]['mean'],
        'worst': single_evo_data[0]['worst']
    }}
    for i in range(0, len(single_evo_data)-1):
        pop = single_evo_data[i]
        next_pop = single_evo_data[i+1]

        best_m = (next_pop['best'] - pop['best']) / (next_pop['num_sims'] - pop['num_sims'])
        best_q = pop['best'] - best_m * pop['num_sims']

        mean_m = (next_pop['mean'] - pop['mean']) / (next_pop['num_sims'] - pop['num_sims'])
        mean_q = pop['mean'] - mean_m * pop['num_sims']

        worst_m = (next_pop['worst'] - pop['worst']) / (next_pop['num_sims'] - pop['num_sims'])
        worst_q = pop['worst'] - worst_m * pop['num_sims']

        for num_sims in range(pop['num_sims']+1, next_pop['num_sims']+1):
            if num_sims <= limit:
                final_evo_data[num_sims] = {
                    'best': best_m*num_sims + best_q,
                    'mean': mean_m*num_sims + mean_q,
                    'worst': worst_m*num_sims + worst_q
                }

    if max(final_evo_data.keys()) < limit:
        print('\t\t\tFitness trends extension - {}'.format(file))
        for num_sims in range(max(final_evo_data.keys())+1, limit+1):
            final_evo_data[num_sims] = copy.deepcopy(final_evo_data[num_sims-1])

    return final_evo_data


def load_experiment(i, folder, chunked, best_key, best_function, worst_key, worst_function, num_sims_limit,
                    fit_diff, init_fit, invert_fit):
    print('\tExperiment #{}'.format(i))

    # load single run evolution data
    evo_files = [os.path.join(folder, f)
                 for f in os.listdir(folder) if re.match('.*evo.*\.json', f)]
    evo_files.sort(key=lambda f: int(re.split('_|\.', f)[-2]))

    if chunked:
        runs = set([int(re.split('_|\.', f)[-2]) for f in evo_files])
        evo_files_tmp = [[] for _ in runs]
        for f in evo_files:
            evo_files_tmp[int(re.split('_|\.', f)[-2])].append(f)
        evo_files = evo_files_tmp

    # combine data from multiple run in a single place
    multi_evo_best = {}
    multi_evo_mean = {}
    multi_evo_worst = {}
    for j, ef in enumerate(evo_files):
        sed = load_evo_file(j, ef, chunked, best_key, worst_key, num_sims_limit, fit_diff, init_fit, invert_fit)
        for num_sims, data in sed.items():
            if num_sims in multi_evo_best:
                multi_evo_best[num_sims] = multi_evo_best[num_sims] + [data['best']]
                multi_evo_mean[num_sims] = multi_evo_mean[num_sims] + [data['mean']]
                multi_evo_worst[num_sims] = multi_evo_worst[num_sims] + [data['worst']]
            else:
                multi_evo_best[num_sims] = [data['best']]
                multi_evo_mean[num_sims] = [data['mean']]
                multi_evo_worst[num_sims] = [data['worst']]

    # perform statistics on combined data
    evo_best_stats = []
    evo_mean_stats = []
    evo_worst_stats = []
    for num_sims in range(len(multi_evo_best)):
        evo_best_stats.append({
            'num_sims': num_sims,
            'best': best_function(multi_evo_best[num_sims]) if (not invert_fit and not fit_diff)
                    else worst_function(multi_evo_best[num_sims]),
            'mean': np.mean(multi_evo_best[num_sims]),
            'std': np.std(multi_evo_best[num_sims])
        })

        evo_mean_stats.append({
            'num_sims': num_sims,
            'mean': np.mean(multi_evo_mean[num_sims]),
            'std': np.std(multi_evo_mean[num_sims])
        })

        evo_worst_stats.append({
            'num_sims': num_sims,
            'worst': worst_function(multi_evo_worst[num_sims]) if (not invert_fit and not fit_diff)
                     else best_function(multi_evo_worst[num_sims]),
            'mean': np.mean(multi_evo_worst[num_sims]),
            'std': np.std(multi_evo_worst[num_sims])
        })

    evo_best_stats.sort(key=lambda x: x['num_sims'])
    evo_mean_stats.sort(key=lambda x: x['num_sims'])
    evo_worst_stats.sort(key=lambda x: x['num_sims'])

    return evo_best_stats, evo_mean_stats, evo_worst_stats


def plot_evolution(data, out_file, sampling_step, sims_limit, min_val, max_val, plot_markers, max_num_cols_legend,
                   squeezing, h_line_val, y_label):
    matplotlib.rcParams['pdf.fonttype'] = 42
    matplotlib.rcParams['ps.fonttype'] = 42
    font = {'family': 'Source Sans Pro', 'size': 18, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300
    # colors = plt.cm.viridis(np.linspace(0, 1, len(data)))

    data_len = len(data) if not squeezing else len(data)+1
    legend_num_cols = min(data_len, max_num_cols_legend)
    legend_num_rows = math.ceil(data_len/max_num_cols_legend)

    markers = ['^', 's', 'X']
    num_markers = 10
    markevery = math.ceil(sims_limit/(sampling_step*num_markers))

    fig = plt.figure(figsize=(12, 8))
    ax = fig.gca()
    for i, d in enumerate(data):
        # color_index = (i % legend_num_rows)*legend_num_cols + math.floor(i/legend_num_rows)
        add_last = sampling_step != 1 and (len(d[1])-1) % sampling_step != 0
        x = list(np.asarray(d[1])[::sampling_step]) + ([d[1][-1]] if add_last else [])
        y = list(np.asarray(d[2])[::sampling_step]) + ([d[2][-1]] if add_last else [])
        std_y_low = list(np.asarray(d[3])[::sampling_step]) + ([d[3][-1]] if add_last else [])
        std_y_high = list(np.asarray(d[4])[::sampling_step]) + ([d[4][-1]] if add_last else [])
        ax.plot(x, y, label=d[0],
                marker=(markers[i] if (plot_markers and i < len(markers)) else ','), markersize=11,
                markevery=(
                    math.ceil(markevery*((i+1)/len(data))),
                    markevery
                ))  # , color = colors[color_index])
        ax.fill_between(x, std_y_low, std_y_high,  # color = colors[color_index],
                        alpha=0.2, antialiased=True)

    if squeezing:
        ax.hlines(h_line_val, 0, sims_limit, label='Aperture Entrance', linestyles='--', colors='black')

    ax.set_title('Fitness Evolution', size=26, fontweight='normal', pad=(40+17*legend_num_rows))
    ax.set_xlabel('Number of Evaluations', labelpad=15, fontweight='light')
    ax.set_ylabel(y_label, labelpad=10, fontweight='light')
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.set_xlim(0, sims_limit)
    ax.set_ylim(min_val, max_val)
    ax.margins(0)
    ax.grid()
    ax.legend(loc='upper center', borderaxespad=-1.55*legend_num_rows,
              ncol=min(data_len, legend_num_cols), frameon=False)

    plt.savefig(out_file, bbox_inches='tight')
    plt.close()


def main(exps_folder, parent_folders_num, out_file, chunked_evo_files, func_best, fit_diff, init_fit, invert_fit,
         num_sims_limit, sampling_step, min_val, max_val, markers, labels, num_cols_legend, squeezing, h_line_val,
         show_list, owner):
    if func_best not in ['min', 'max']:
        raise Exception('The function provided to determine the best fitness'
                        + 'is not valid')
    else:
        best_key, function_best, worst_key, function_worst = \
            ('min', np.min, 'max', np.max) if func_best == 'min' \
            else ('max', np.max, 'min', np.min)

    if chunked_evo_files == [False]:
        chunked_evo_files = [False for _ in range(len(exps_folder))]
    elif len(chunked_evo_files) != len(exps_folder):
        raise Exception('The number of chunked evo files flags is different from the number of experiments!')
    
    print('Loading experiments and interpolating values...')
    fold_names, evo_best_stats, evo_avg_stats, evo_worst_stats =\
        list(zip(*[(os.path.basename(os.path.normpath(f)),
                    *(load_experiment(i+1, f, chunked, best_key, function_best,
                                      worst_key, function_worst, num_sims_limit,
                                      fit_diff, init_fit, invert_fit)))
                      for i, (f, chunked) in enumerate(zip(exps_folder, chunked_evo_files))]))

    if labels is not None:
        if len(labels) != len(fold_names):
            raise Exception('The number of provided labels'
                            + 'is different from the number'
                            + f'of experiments: {len(labels)}!={len(fold_names)}')

    if sampling_step <= 0:
        raise Exception('The sampling step provided is not valid! It must be > 0')

    label_prop = len(show_list) != 1 or 'all' in show_list

    exps_best = []
    exps_best_avg = []
    exps_avg = []
    exps_worst = []
    exps_worst_avg = []
    print('Extracting statistics...')
    for i, (best_data, avg_data, worst_data) in enumerate(zip(evo_best_stats, evo_avg_stats, evo_worst_stats)):
        sims, best_fit, avg_best_fit, std_best_fit =\
            list(zip(*[(pop['num_sims'], pop['best'], pop['mean'], pop['std']) for pop in best_data]))

        avg_avg_fit, std_avg_fit = \
            list(zip(*[(pop['mean'], pop['std']) for pop in avg_data]))

        worst_fit, avg_worst_fit, std_worst_fit = \
            list(zip(*[(pop['worst'], pop['mean'], pop['std']) for pop in worst_data]))

        labels_prefix = labels[i] if labels is not None else ''

        exps_best.append([labels_prefix+(' Best' if label_prop else ''), sims, best_fit, best_fit, best_fit])

        exps_best_avg.append([labels_prefix+(' Avg. Best' if label_prop else ''), sims, avg_best_fit,
                             np.asarray(avg_best_fit) - np.asarray(std_best_fit),
                             np.asarray(avg_best_fit) + np.asarray(std_best_fit)])

        exps_avg.append([labels_prefix+(' Avg.' if label_prop else ''), sims, avg_avg_fit,
                         np.asarray(avg_avg_fit) - np.asarray(std_avg_fit),
                         np.asarray(avg_avg_fit) + np.asarray(std_avg_fit)])

        exps_worst.append([labels_prefix+(' Worst' if label_prop else ''), sims, worst_fit, worst_fit, worst_fit])

        exps_worst_avg.append([labels_prefix+(' Avg. Worst' if label_prop else ''), sims, avg_worst_fit,
                              np.asarray(avg_worst_fit) - np.asarray(std_worst_fit),
                              np.asarray(avg_worst_fit) + np.asarray(std_worst_fit)])

    parent_folder = os.path.normpath(exps_folder[0])
    for i in range(0, parent_folders_num):
        parent_folder = os.path.dirname(parent_folder)
    out_folder = os.path.join(parent_folder, 'evo_plots')
    os.makedirs(out_folder, exist_ok=True)

    print('Plotting results...')
    y_label_prefix = []
    if 'all' in show_list:
        plot_data = exps_best_avg + exps_avg + exps_worst_avg
    else:
        plot_data = []
        if 'best' in show_list:
            plot_data += exps_best
            y_label_prefix.append('Best ')
        if 'best-avg' in show_list:
            plot_data += exps_best_avg
            y_label_prefix.append('Avg. Best ')
        if 'avg' in show_list:
            plot_data += exps_avg
            y_label_prefix.append('Avg. ')
        if 'worst' in show_list:
            plot_data += exps_worst
            y_label_prefix.append('Worst ')
        if 'worst-avg' in show_list:
            plot_data += exps_worst_avg
            y_label_prefix.append('Avg. Worst ')

    if len(plot_data) > 0:
        y_label_prefix = y_label_prefix[0] if len(y_label_prefix) == 1 else ''
        plot_evolution(plot_data, os.path.join(out_folder, out_file), sampling_step, num_sims_limit, min_val, max_val,
                       markers, num_cols_legend, squeezing, h_line_val, y_label=y_label_prefix+'Fitness [cm]')

        if owner is not None and len(owner) == 2:
            try:
                exec_string = 'chown -R {}:{} {}'.format(owner[0], owner[1], out_folder)
                c_proc = subprocess.run(exec_string.split(' '), capture_output=True)
            except:
                raise Exception('An error occurred during the owner setting')
    else:
        print('No valid property specified in show list!')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for plotting the progress and results'
                                                 'of the evolutionary algorithm over different experiments')
    parser.add_argument('exps_folders', metavar='exps_folders', type=str, nargs='+',
                        help='list of experiments folders containing evolution info files')
    parser.add_argument('--parent-folders-num', metavar='parent-folders-num', type=int, action='store',
                        default=1, help='number of parent folders to traverse before creating the out folder')
    parser.add_argument('--out-file', metavar='out-file', type=str, action='store',
                        default='evo_comp.pdf', help='output filename')
    parser.add_argument('--chunked-evo-files', metavar='chunked-evo-files', type=str2bool, nargs='+',
                        default=[False], help='which of these experiments contain chunked evo files?')
    parser.add_argument('--fit-func-best', metavar='fitness-function-best', type=str, nargs='?', default='min',
                        help='function used to determine the best fitness, allowed values are: min, max')
    parser.add_argument('--fit-difference', dest='fit_difference', action='store_const',
                        const=True, default=False, help='show fitness values as (init_fit - fit_value)')
    parser.add_argument('--init-fit', metavar='init-fit', type=float, action='store',
                        default=45, help='initial fit value, used in combination with fit-difference')
    parser.add_argument('--invert-fit', dest='invert_fit', action='store_const',
                        const=True, default=False, help='invert fitness values')
    parser.add_argument('--num-sims', metavar='num-sims', type=float, action='store',
                        default=45000, help='maximum number of simulations')
    parser.add_argument('--sampling-step', metavar='sampling-step', type=int, nargs='?',
                        default=1, help='data sampling step for plotting')
    parser.add_argument('--min-val', metavar='min-val', type=float, action='store',
                        default=0, help='minimum fitness value provided for visualization')
    parser.add_argument('--max-val', metavar='max-val', type=float, action='store',
                        default=60, help='maximum fitness value provided for visualization')
    parser.add_argument('--markers', dest='markers', action='store_const',
                        const=True, default=False, help='plot markers')
    parser.add_argument('--labels', metavar='labels', type=str, action='store', nargs='+',
                        default=None, help='charts labels - must match the number of experiments!')
    parser.add_argument('--num-cols-legend', metavar='num-cols-legend', type=int, action='store', nargs='?',
                        default=3, help='number of columns of chart legend')
    parser.add_argument('--squeezing', dest='squeezing', action='store_const',
                        const=True, default=False, help='plot h-line for squeezing experiments (aperture entrance)')
    parser.add_argument('--h-line-val', metavar='h-line-val', type=float, nargs='?',
                        default=44.83, help='horizontal line value (fitness limit for squeezing)')
    parser.add_argument('--show', metavar='show', type=str, action='store', nargs='+',
                        default='all', help='list of properties to show in the chart, allowed values are: '
                                            'all (includes best-avg, avg and worst-avg), avg, best, best-avg, '
                                            'worst, worst-avg')
    parser.add_argument('--owner', metavar='owner', type=str, action='store', nargs='+',
                        default=None, help='User and group for chown')

    args = parser.parse_args()
    main(args.exps_folders, args.parent_folders_num, args.out_file, args.chunked_evo_files, args.fit_func_best,
         args.fit_difference, args.init_fit, args.invert_fit, args.num_sims, args.sampling_step,
         args.min_val, args.max_val, args.markers, args.labels, args.num_cols_legend, args.squeezing, args.h_line_val,
         args.show, args.owner)
