import argparse
import json
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import os
import re
import sys
import warnings

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from goal_reaching.statistics_reporter import StatisticsReporterGR


def plot_species(stats_checkpoint_file, res_dir, seed):
    most_fit_genomes, generation_statistics = StatisticsReporterGR.restore_checkpoint(stats_checkpoint_file)
    statistics = StatisticsReporterGR(most_fit_genomes=most_fit_genomes, generation_statistics=generation_statistics)

    filename = os.path.join(
        res_dir,
        'evolution_info',
        'species_{}.png'.format(seed)
    )

    if plt is None:
        warnings.warn("This display is not available due to a missing optional dependency (matplotlib)")
        return

    species_sizes = statistics.get_species_sizes()
    num_generations = len(species_sizes)
    curves = np.array(species_sizes).T

    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    fig = plt.figure(figsize=(8, 5))
    ax = fig.gca()
    ax.stackplot(range(num_generations), *curves)

    ax.set_title("Speciation")
    ax.set_ylabel("Size per Species")
    ax.set_xlabel("Generations")

    plt.savefig(filename)
    plt.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for plotting NEAT species sizes throughout generations.')
    parser.add_argument('res_dir', metavar='res_dir', type=str, nargs='?',
                        default='res_dir', help='the folder where results are stored')
    args = parser.parse_args()

    settings_file = os.path.join(args.res_dir, 'settings.json')
    with open(settings_file) as sf:
        settings = json.load(sf)

    checkpoint_dir = os.path.join(args.res_dir, 'checkpoints')
    for seed in settings['seeds']:
        species_files = {int(f.split('_')[-1]): os.path.join(checkpoint_dir, f)
                         for f in os.listdir(checkpoint_dir)
                         if re.match('contr_stats_cp_{}_[0-9]+'.format(seed), f)}
        species_file = species_files[max(species_files.keys())]

        plot_species(species_file, args.res_dir, seed)
