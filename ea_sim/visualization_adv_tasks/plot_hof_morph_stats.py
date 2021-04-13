import argparse
import json
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from utils import parse_robot_string


def plot_hof_morph_stats(hall_of_fame_file, morphologies_file, res_dir, seed):
    # create results directory
    hof_stats_dir = os.path.join(res_dir, 'hall_of_fame_stats')
    os.makedirs(hof_stats_dir, exist_ok=True)

    with open(hall_of_fame_file) as hof_file:
        hall_of_fame = json.load(hof_file)

    with open(morphologies_file) as morph_file:
        morphologies = json.load(morph_file)
    inverse_morph_dict = {v: k for k, v in morphologies.items()}

    fitnesses = []
    num_modules = []
    stiffnesses = []
    for fitness, pair_list in hall_of_fame.items():
        for m_id, c_id in pair_list:
            robot = parse_robot_string(inverse_morph_dict[m_id])

            fitnesses.append(float(fitness))
            num_modules.append(len(robot))
            stiffnesses.append(robot[0]['stiff'])

    confs = [
        {
            'data': fitnesses,
            'title': 'Fitness',
            'h_range': (np.min(fitnesses), np.max(fitnesses)),
            'out_file': os.path.join(
                hof_stats_dir,
                'fitness_{}.pdf'.format(seed)
            ),
            'bins': max(len(fitnesses) // 8, 1),
            'discrete_hist': False,
            'norm': False
        },
        {
            'data': num_modules,
            'title': '# modules',
            'h_range': (2, 11),
            'out_file':  os.path.join(
                hof_stats_dir,
                'n_modules_{}.pdf'.format(seed)
            ),
            'bins': list(range(2, 11)),
            'discrete_hist': True,
            'norm': False
        },
        {
            'data': stiffnesses,
            'title': 'Stiffness',
            'out_file':  os.path.join(
                hof_stats_dir,
                'stiffness_{}.pdf'.format(seed)
            ),
            'bins': [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0],
            'discrete_hist': True,
            'norm': False
        }
    ]

    for conf in confs:
        plot_stats(conf)


def plot_stats(conf):
    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    num_colors = len(conf['bins']) if isinstance(conf['bins'], list) else conf['bins']
    colors = plt.cm.viridis(np.linspace(0, 1, num_colors))

    fig = plt.figure(figsize=(12, 5))
    ax = fig.gca()

    if conf['discrete_hist']:
        inds_vals = {str(sv): 0 for sv in conf['bins']}
        for ind_stiff in conf['data']:
            inds_vals[str(ind_stiff)] += 1
        x, y = list(zip(*[(k, v) for k, v in sorted(inds_vals.items(), key=lambda r: float(r[0]))]))
        if conf['norm']:
            y = np.asarray(y) / np.sum(y)
        ax.bar(x, y, color=colors)
    else:
        _, bins, patches = ax.hist(conf['data'], bins=conf['bins'], density=conf['norm'],
                                   range=conf['h_range'])
        ax.set_xticks(bins)
        for i, (c, p) in enumerate(zip(bins, patches)):
            plt.setp(p, 'facecolor', colors[i])

    ax.set_title('{} distribution across last generation'.format(conf['title']),
                 fontweight='normal')
    ax.set_xlabel(conf['title'])
    ax.set_ylabel('# Individuals')

    plt.savefig(conf['out_file'], bbox_inches='tight')
    plt.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for plotting statistics (mainly morphological) about the '
                                                 'individuals contained in the hall of fame (fitness, '
                                                 '# of modules and stiffness distributions).')
    parser.add_argument('res_dir', metavar='res_dir', type=str, nargs='?',
                        default='res_dir', help='the folder where results are stored')
    args = parser.parse_args()

    settings_file = os.path.join(args.res_dir, 'settings.json')
    with open(settings_file) as sf:
        settings = json.load(sf)

    for seed in settings['seeds']:
        hall_of_fame_file = os.path.join(args.res_dir, 'best', 'hall_of_fame_{}.json'.format(seed))
        morphologies_file = os.path.join(args.res_dir, 'morphologies', 'morphologies_{}.json'.format(seed))

        plot_hof_morph_stats(hall_of_fame_file, morphologies_file, args.res_dir, seed)
