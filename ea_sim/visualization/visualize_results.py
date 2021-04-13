#! /usr/bin/env python3

## OLD SCRIPT - NO MORE USED ##

import os
import re
import json
import argparse
import numpy as np

import matplotlib
# select matplotlib backend
matplotlib.use('pdf')
import matplotlib.pyplot as plt

def prepare_env(results_folder):
    # collect which experiments should be visualized
    experiment_folders = [f for f in os.listdir(results_folder)
                          if re.match('^(s)?[0-9]+\_.*', f)]

    # prepare output folder
    plots_folder = os.path.join(results_folder, 'plots')
    if not os.path.exists(plots_folder):
        os.mkdir(plots_folder, os.O_RDWR)

    return experiment_folders, plots_folder


def load_json(json_file):
    with open(json_file) as raw_data:
        exp_data = json.load(raw_data)

    return exp_data


def store_json(data, json_file):
    os.makedirs(os.path.dirname(json_file), exist_ok=True)
    with open(json_file, 'w') as out_file:
        json.dump(data, out_file)

def compute_stats(data):
    return {
        'median': np.median(data),
        'avg': np.mean(data),
        'std': np.std(data),
        'min': np.min(data),
        'max': np.max(data),
        'q1': np.percentile(data, 25),
        'q3': np.percentile(data, 75)
    }


def sum_stats(s1, s2):
    return {
        'median': s1['median'] + s2['median'],
        'avg': s1['avg'] + s2['avg'],
        'std': s1['std'] + s2['std'],
        'min': min(s1['min'], s2['min']),
        'max': max(s1['max'], s2['max']),
        'q1': s1['q1'] + s2['q1'],
        'q3': s1['q3'] + s2['q3']
    }


def divide_stats(s, divisor):
    return {
        'median': float(s['median']/divisor),
        'avg': float(s['avg']/divisor),
        'std': float(s['std']/divisor),
        'min': float(s['min']),
        'max': float(s['max']),
        'q1': float(s['q1']/divisor),
        'q3': float(s['q3']/divisor)
    }

def process_sims_data(sim_files, results_folder, exp):
    # variable for storing all the statistics of each monitored property
    # for each generation of each simulation
    final_fits = []

    sims_fit_values = []
    for j, sim in enumerate(sim_files):
        generations = load_json(os.path.join(results_folder, exp, sim))

        sims_fit_values.append([max([ind['fitness'] for ind in gen['population']])
                                for gen in generations])

    sims_fit_values = np.asarray(sims_fit_values)
    mean_fit = []
    medians_fit = []
    std_fit = []
    q1s = []
    q3s = []

    # take the values (median, Q1, Q3) from all the different simulations (different seeds)
    for gen in range(len(sims_fit_values[0])):
        mean_fit.append(np.mean(sims_fit_values[:, gen]))
        medians_fit.append(np.median(sims_fit_values[:, gen]))
        std_fit.append(np.std(sims_fit_values[:, gen]))
        q1s.append(np.percentile(sims_fit_values[:, gen], 25))
        q3s.append(np.percentile(sims_fit_values[:, gen], 75))

    return mean_fit, medians_fit, std_fit, q1s, q3s


def augment_data_map(data):
    # first 25 simulation checkpoints, MAP elites does not produce anything
    # therefore zero data point are inserted
    new_data = ([0.0]*25, [0.0]*25, [0.0]*25, [0.0]*25, [0.0]*25)

    # copy each data point, duplicating the first and second ones every three points
    i = 0
    while i < len(data[0]):
        for j in range(len(data)):
            new_data[j].append(data[j][i])

            if i % 3 != 0:
                new_data[j].append(data[j][i])
                new_data[j].append(data[j][i])
            if i % 3 == 0 and i % 6 == 0:
                new_data[j].append(data[j][i])

        i += 1

    return new_data


def plot_experiments(medians, q1s, q3s, config, title='',
                     colors=None, y_max=40, y_label='Median Max Fitness [cm]'):
    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    fig = plt.figure(figsize=(12, 8))
    ax = fig.gca()

    if colors is None:
        colors = plt.cm.viridis(np.linspace(0, 1, len(medians)))

    for i in range(len(medians)):
        ax.plot(list(range(len(medians[i]))), medians[i],
                label=config['labels'][i],
                color=colors[i], ms=2.5)
        ax.fill_between(list(range(len(q1s[i]))),
                        q1s[i],
                        q3s[i],
                        color=colors[i],
                        alpha=0.2,
                        antialiased=True)

    ax.set_title(title, size=20, fontweight='normal')
    ax.set_xlabel('Simulations Checkpoints', labelpad=15)
    ax.set_ylabel(y_label, labelpad=10)
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.set_xlim(0, max([len(m) for m in medians]))
    ax.set_ylim(0, y_max)
    ax.margins(0)
    ax.grid()
    ax.legend(loc=8, borderaxespad=1, ncol=4, frameon=False)

    out_file = '{}.pdf'.format(os.path.join(config['base_dir'], config['output_file']))
    os.makedirs(os.path.dirname(out_file), exist_ok=True)
    plt.savefig(out_file, bbox_inches='tight')
    plt.close()

def _get_null_values():
    return [([0.0], [0.0], [0.0])]


def main(results_folder):
    exps_folders, plots_folder = prepare_env(results_folder)

    # process each experiment according to its numerical order
    exps_vals = {}
    colors = plt.cm.viridis(np.linspace(0, 1, len(exps_folders)))
    for i, exp in enumerate(exps_folders):
        print('\rProcessing experiment: {}'.format(i + 1), end='')

        # collect each simulation file
        sim_files = [f for f in os.listdir(os.path.join(results_folder, exp))
                     if re.match('^evo.*\.json', f)]

        # load data into an appropriate data structure
        mean, median, std, q1, q3 = process_sims_data(sim_files, results_folder, exp)

        exps_vals[exp] = (mean, median, std, q1, q3)

        # consider the different method for check-pointing in MAP-Elites
        if exp == '17_map-elites_selected_noiseless' or exp == '18_map-elites_selected_noisy':
            exps_vals[exp] = augment_data_map(exps_vals[exp])

        exp_comps = exp.split('_')
        title = '{} {}'.format(exp_comps[1], exp_comps[-1])
        config = {
            'labels': [title],
            'base_dir': plots_folder,
            'output_file': '_'.join(exp_comps[:2] + [exp_comps[-1]] + ['fit'])
        }
        plot_experiments([median], [q1], [q3], config, title)

    #noiseless_exps = ['01_vie_selected_noiseless', '09_mu+l_selected_noiseless', '17_map-elites_selected_noiseless']
    noisy_exps = ['05_vie_selected_noisy', '13_mu+l_selected_noisy', '18_map-elites_selected_noisy']

    # noiseless_medians, noiseless_q1s, noiseless_q3s =\
    #     list(zip(*[exps_vals[nle] if nle in exps_folders else _get_null_values() for nle in noiseless_exps]))
    n_means, n_medians, n_stds, n_q1s, n_q3s =\
        list(zip(*[exps_vals[nye] if nye in exps_folders else _get_null_values() for nye in noisy_exps]))

    print('\nPlot EAs comparisons...')
    # plot_experiments(noiseless_medians,
    #                  noiseless_q1s,
    #                  noiseless_q3s,
    #                  {
    #                      'labels': noiseless_exps,
    #                      'base_dir': plots_folder,
    #                      'output_file': 'alg_comparison_noiseless'
    #                  },
    #                  'EAs comparison - Noiseless'
    #                  )
    plot_experiments(n_medians,
                     n_q1s,
                     n_q3s,
                     {
                         'labels': noisy_exps,
                         'base_dir': plots_folder,
                         'output_file': 'alg_comparison_median'
                     },
                     'EAs comparison'
                     )
    plot_experiments(n_means,
                     [np.asarray(m) - np.asarray(s) for m, s in zip(n_means, n_stds)],
                     [np.asarray(m) + np.asarray(s) for m, s in zip(n_means, n_stds)],
                     {
                         'labels': noisy_exps,
                         'base_dir': plots_folder,
                         'output_file': 'alg_comparison_avg'
                     },
                     'EAs comparison'
                     , y_label='Avg. Max Fitness [cm]'
                     )
    print('Done!')

if __name__ == '__main__':
    print('\n## OLD SCRIPT - NO MORE USED ##\n')
    parser = argparse.ArgumentParser(description='Script for plotting the progress and results'
                                                 'of the evolutionary algorithm over different experiments')
    parser.add_argument('in_folder', metavar='in_folder', type=str, nargs='?',
                        default='../results', help='folder where experiments data are stored.')

    args = parser.parse_args()
    main(args.in_folder)