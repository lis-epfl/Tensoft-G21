#! /usr/bin/env python3

import os
import re
import json
import argparse
import numpy as np

import matplotlib
# select matplotlib backend
matplotlib.use('pdf')
import matplotlib.pyplot as plt


def load_evo_file(file, total_num_sim=None):
    with open(file) as evo_file:
        raw_data = json.load(evo_file)

    single_evo_data = {}
    for pop in raw_data:
        single_evo_data[pop['num_sims']] = np.asarray([ind['fitness'] for ind in pop['population']])

    if total_num_sim is not None:
        last_sim = raw_data[-1]['num_sims']
        step = last_sim-raw_data[-2]['num_sims']

        if last_sim < total_num_sim:
            last_val = single_evo_data[last_sim]

            # convert the maximum number of simulations into a multiple of the gen step (pop size)
            total_num_sim = int(np.ceil(total_num_sim / step) * step)

            for curr_sim in range(last_sim+step, total_num_sim+step, step):
                single_evo_data[curr_sim] = np.copy(last_val)

    return single_evo_data


def load_experiment(folder, sims_budget=None):
    # load single run evolution data
    evo_files = [os.path.join(folder, f)
                 for f in os.listdir(folder) if re.match('^evo.*\.json', f)]

    # combine data from multiple run in a single place
    multi_evo_data = {}
    multi_evo_max = {}
    for ef in evo_files:
        sed = load_evo_file(ef, sims_budget)
        for nsims, pop in sed.items():
            if nsims in multi_evo_data:
                multi_evo_data[nsims] = np.concatenate((multi_evo_data[nsims], pop))
                multi_evo_max[nsims] = multi_evo_max[nsims] + [np.max(pop)]
            else:
                multi_evo_data[nsims] = pop
                multi_evo_max[nsims] = [np.max(pop)]

    # perform statistics on combined data
    evo_stats = []
    evo_stats_max = []
    for nsims, fit_vals in multi_evo_data.items():
        evo_stats.append({
            'num_sims': nsims,
            'mean': np.mean(fit_vals),
            'std': np.std(fit_vals),
            'min': np.min(fit_vals),
            'max': np.max(fit_vals),
            'median': np.median(fit_vals),
            'q1': np.percentile(fit_vals, 25),
            'q3': np.percentile(fit_vals, 75)
        })

    # this is necessary to compute the average among the maximum fitness value
    # of the different runs throughout the evolution process
    for nsims, max_vals in multi_evo_max.items():
        evo_stats_max.append({
            'num_sims': nsims,
            'mean': np.mean(max_vals),
            'std': np.std(max_vals),
        })

    evo_stats.sort(key=lambda x: x['num_sims'])
    evo_stats_max.sort(key=lambda x: x['num_sims'])

    return evo_stats, evo_stats_max


def plot_evolution(data, out_file, max_fit, y_label):
    font = {'family': 'Source Sans Pro', 'size': 18, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300
    colors = plt.cm.viridis(np.linspace(0, 1, len(data)))

    fig = plt.figure(figsize=(12, 8))
    ax = fig.gca()
    for i, d in enumerate(data):
        ax.plot(d[1], d[2], label=d[0], color=colors[i])
        ax.fill_between(d[1], d[3], d[4], color=colors[i],
                        alpha=0.2, antialiased=True)

    ax.set_title('Fitness Evolution', size=26, fontweight='normal', pad=40)
    ax.set_xlabel('Simulations', labelpad=15, fontweight='light')
    ax.set_ylabel(y_label, labelpad=10, fontweight='light')
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.set_xlim(0, max([d[1][-1] for d in data])) # set to the max num of sims of first exp
    ax.set_ylim(0, max_fit)
    ax.margins(0)
    ax.grid()
    ax.legend(loc='upper center', borderaxespad=-2, ncol=len(data), frameon=False)

    plt.savefig(out_file, bbox_inches='tight')
    plt.close()


def main(exps_folder, out_file, max_val, labels, sims_budget=None):
    print('Loading experiments...')
    fold_names, evo_statistics, evo_s_max =\
        list(zip(*[(os.path.basename(os.path.normpath(f)),
                    *(load_experiment(f, sims_budget))) for f in exps_folder]))

    if labels is not None:
        if len(labels) != len(fold_names):
            raise Exception('The number of provided labels'
                            + 'is different from the number'
                            + f'of experiments: {len(labels)}!={len(fold_names)}')
    else:
        labels = fold_names

    exps_avg = []
    exps_med = []
    exps_max = []
    exps_max_avg = []
    print('Extracting statistics...')
    for i, st in enumerate(evo_statistics):
        sims, avg_fit, std_fit, median_fit, q1_fit, q3_fit, max_fit =\
            list(zip(*[(pop['num_sims'], pop['mean'], pop['std'], pop['median'],
                        pop['q1'], pop['q3'], pop['max']) for pop in st]))

        exps_avg.append([labels[i], sims, avg_fit, np.asarray(avg_fit) - np.asarray(std_fit),
                         np.asarray(avg_fit) + np.asarray(std_fit)])
        exps_med.append([labels[i], sims, median_fit, q1_fit, q3_fit])
        exps_max.append([labels[i], sims, max_fit, max_fit, max_fit])

        print(labels[i], f'avg: {avg_fit[-1]}', f'std {std_fit[-1]}',
              f'median: {median_fit[-1]}', f'max: {max_fit[-1]}')

    # extract the average max fitness from each experiment
    for i, st in enumerate(evo_s_max):
        sims, avg_max, std_max = list(zip(*[(pop['num_sims'], pop['mean'], pop['std']) for pop in st]))

        exps_max_avg.append([labels[i], sims, avg_max, np.asarray(avg_max) - np.asarray(std_max),
                            np.asarray(avg_max) + np.asarray(std_max)])

        print('Max-avg ->', labels[i], f'avg: {avg_max[-1]}', f'std {std_max[-1]}')

    parent_folder = os.path.dirname(os.path.normpath(exps_folder[0]))
    out_folder = os.path.join(parent_folder, 'evo_plots')
    os.makedirs(out_folder, exist_ok=True)

    print('Plotting results...')
    plot_evolution(exps_avg, os.path.join(out_folder, f'avg_{out_file}'),
                   max_val, y_label='Avg. Fitness [cm]')
    plot_evolution(exps_med, os.path.join(out_folder, f'med_{out_file}'),
                   max_val, y_label='Median Fitness [cm]')
    plot_evolution(exps_max, os.path.join(out_folder, f'max_{out_file}'),
                   max_val, y_label='Max Fitness [cm]')
    plot_evolution(exps_max_avg, os.path.join(out_folder, f'max_avg_{out_file}'),
                   max_val, y_label='Avg. Max Fitness [cm]')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for plotting the progress and results'
                                                 'of the evolutionary algorithm over different experiments')
    parser.add_argument('exps_folder', metavar='exps_folder', type=str, nargs='+',
                        help='list of experiments folders to be visualized')
    parser.add_argument('--out-file', metavar='out-file', type=str, action='store',
                        default='evo_comp.pdf', help='output filename where merged history is stored')
    parser.add_argument('--labels', metavar='labels', type=str, action='store', nargs='+',
                        default=None, help='charts labels - must match the number of experiments!')
    parser.add_argument('--max-fit', metavar='max-fit', type=float, action='store',
                        default=60, help='maximum fitness value provided for visualization')
    parser.add_argument('--sims-budget', metavar='sims_budget', type=int, action='store',
                        default=None, help='the number of simulations to which an experiment should be'
                                           'extended at in case it has converged earlier')

    args = parser.parse_args()
    main(args.exps_folder, args.out_file, args.max_fit, args.labels, args.sims_budget)
