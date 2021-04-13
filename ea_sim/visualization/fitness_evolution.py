import os
import json
import argparse
import matplotlib
import numpy as np

import matplotlib.pyplot as plt


def plot(evo_file, chunked_file=False):
    if not chunked_file:
        with open(evo_file) as in_file:
            evo_data = json.load(in_file)

        # iterate over all saved population and collect
        # in different list the 4 interested properties
        sims, avg_fit, min_fit, max_fit =\
            list(zip(*[(pop['num_sims'], pop['avg_fitness'], pop['min'], pop['max'])
                       for pop in evo_data]))
    else:
        sims, avg_fit, min_fit, max_fit = [], [], [], []
        for evo_file_chunk in evo_file:
            print('\t{}'.format(evo_file_chunk))
            with open(evo_file_chunk) as in_file:
                evo_data = json.load(in_file)

                sims_chunk, avg_fit_chunk, min_fit_chunk, max_fit_chunk = \
                    list(zip(*[(pop['num_sims'], pop['avg_fitness'], pop['min'], pop['max'])
                               for pop in evo_data]))

                evo_data = []
                in_file.close()

                sims += list(sims_chunk)
                avg_fit += list(avg_fit_chunk)
                min_fit += list(min_fit_chunk)
                max_fit += list(max_fit_chunk)

    # plot fitness evolution
    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300
    colors = plt.cm.viridis(np.linspace(0, 1, 3))

    fig = plt.figure(figsize=(12, 8))
    ax = fig.gca()

    ax.plot(sims, avg_fit, label='Avg. Fitness', color=colors[0])
    ax.plot(sims, min_fit, label='Min. Fitness', color=colors[1])
    ax.plot(sims, max_fit, label='Max. Fitness', color=colors[2])

    ax.set_title('Fitness Evolution', size=20, fontweight='normal')
    ax.set_xlabel('Simulations', labelpad=15)
    ax.set_ylabel('Fitness Value', labelpad=10)
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.set_xlim(0, sims[-1])
    ax.set_ylim(0, max(max_fit)+1)
    ax.margins(0)
    ax.grid()
    ax.legend(loc=8, borderaxespad=1, ncol=3, frameon=False)

    out_file = evo_file.replace('json', 'pdf') if not chunked_file else evo_file[0].replace('.json', '.pdf')
    os.makedirs(os.path.dirname(os.path.normpath(out_file)), exist_ok=True)
    plt.savefig(out_file, bbox_inches='tight')
    plt.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for plotting population fitness evolution across the time.')
    parser.add_argument('evo_file', metavar='evo_file', type=str, nargs='?',
                        default='evo_json_213_sim_0.json',
                        help='the folder where results archives are stored')

    args = parser.parse_args()
    plot(args.evo_file)
