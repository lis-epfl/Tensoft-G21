import argparse
import json
import neat
import os
import subprocess
import re
import sys

import matplotlib
# select matplotlib backend
matplotlib.use('pdf')
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from goal_reaching.utils_gr import parse_controller
from utils import parse_robot_string
from visualization_adv_tasks.plot_hof_controllers_stats import compute_rates


def load_hof_contr_stats(i, hall_of_fame_file, morphologies_file, neat_settings, controllers_dir,
                         seed, max_num_individuals):
    print('\t\tRun #{}'.format(i))

    with open(hall_of_fame_file) as hof_file:
        hall_of_fame = json.load(hof_file)

    with open(morphologies_file) as morph_file:
        morphologies = json.load(morph_file)
    inverse_morph_dict = {v: k for k, v in morphologies.items()}

    neat_config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                              neat.DefaultSpeciesSet, neat.DefaultStagnation,
                              neat_settings)

    counter = 0
    controllers = []
    for pair_list in hall_of_fame.values():
        for m_id, c_id in pair_list:
            genome, _, _ = parse_controller(
                os.path.join(controllers_dir, 'controller_{}_{}.txt'.format(seed, c_id)),
                c_id
            )
            controller_nn = neat.nn.FeedForwardNetwork.create(genome, neat_config)
            morphology = parse_robot_string(inverse_morph_dict[m_id])
            controllers.append((genome, controller_nn, len(morphology)))

            counter += 1

            if counter == max_num_individuals:
                break
        if counter == max_num_individuals:
            break

    i_o_rates, i_h_rates, h_h_rates, h_o_rates, o_h_rates, o_o_rates = compute_rates(controllers)

    return i_o_rates, i_h_rates, h_h_rates, h_o_rates, o_h_rates, o_o_rates


def load_experiment(i, folder, max_num_individuals):
    print('\tExperiment #{}'.format(i))

    # load single run hall of fame stats
    hall_of_fames = [
        (
            os.path.join(folder, f),
            os.path.join(folder, f.replace('.json', ''), 'morphologies_{}.json'.format(f.split('_')[-2])),
            os.path.join(folder, f.replace('.json', ''), 'neat_settings.txt'),
            os.path.join(folder, f.replace('.json', ''), 'controllers'),
            int(f.split('_')[-2])
        )
        for f in os.listdir(folder) if re.match('hall_of_fame.*\.json', f)
    ]
    hall_of_fames.sort(key=lambda element: int(re.split('_|\.', element[0])[-2]))

    i_o_rates_lists, i_h_rates_lists, h_h_rates_lists, h_o_rates_lists, o_h_rates_lists, o_o_rates_lists = \
        list(zip(*[load_hof_contr_stats(j, hof_f, morph_f, neat_f, contr_d, seed, max_num_individuals)
                   for j, (hof_f, morph_f, neat_f, contr_d, seed) in enumerate(hall_of_fames)]))

    i_o_rates = [rate for rates in i_o_rates_lists for rate in rates]
    i_h_rates = [rate for rates in i_h_rates_lists for rate in rates]
    h_h_rates = [rate for rates in h_h_rates_lists for rate in rates]
    h_o_rates = [rate for rates in h_o_rates_lists for rate in rates]
    o_h_rates = [rate for rates in o_h_rates_lists for rate in rates]
    o_o_rates = [rate for rates in o_o_rates_lists for rate in rates]

    return [i_o_rates, i_h_rates, h_h_rates, h_o_rates, o_h_rates, o_o_rates]


def plot_boxplots(conf):
    font = {'family': 'Source Sans Pro', 'size': 18, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    if len(conf['data']) == 1:
        fig = plt.figure(figsize=(12, 8))
        ax = fig.gca()
        ax.boxplot(conf['data'][0])

        ax.set_xticklabels(conf['x_tick_labels'])
        ax.set_xlabel(conf['x_labels'][0], labelpad=7, fontweight='light', fontsize=18)
        ax.set_ylabel(conf['y_label'], labelpad=3, fontweight='light', fontsize=18)
        ax.set_ylim(-0.1, 1.1)
    else:
        fig, axes = plt.subplots(1, len(conf['data']), sharey='all', figsize=(4*len(conf['data']), 4))
        fig.suptitle(conf['title'], y=0.95, fontweight='normal', fontsize=18)

        for ax, (i, d) in zip(axes, enumerate(conf['data'])):
            ax.boxplot(d)

            ax.set_xticklabels(conf['x_tick_labels'])
            ax.set_xlabel(conf['x_labels'][i], labelpad=7, fontweight='light', fontsize=18)
            ax.set_ylim(-0.1, 1.1)

        axes[0].set_ylabel(conf['y_label'], labelpad=3, fontweight='light', fontsize=18)
        # fix y ticks
        for ax in axes[1:]:
            ax.tick_params(axis='y', which='both', length=0)
        plt.subplots_adjust(wspace=0.04)

    plt.savefig(conf['out_file'], bbox_inches='tight')
    plt.close()


def main(exps_folders, max_num_individuals, parent_folders_num, out_file, labels, owner):
    print('Loading experiments...')
    plot_data = [load_experiment(i + 1, f, max_num_individuals)
                 for i, f in enumerate(exps_folders)]

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

    print('Plotting results...')
    conf = {
        'data': plot_data,
        'title': 'Connections usage in Hall of Fame controllers' +
                 (' (size={})'.format(max_num_individuals) if max_num_individuals != 101 else '') + ' across runs',
        'x_tick_labels': [
            'I-O', 'I-H', 'H-H', 'H-O', 'O-H', 'O-O'
        ],
        'x_labels': labels,
        'y_label': 'Conn. usage ratio',
        'out_file': os.path.join(out_folder, out_file)
    }
    plot_boxplots(conf)

    if owner is not None and len(owner) == 2:
        try:
            exec_string = 'chown -R {}:{} {}'.format(owner[0], owner[1], out_folder)
            c_proc = subprocess.run(exec_string.split(' '), capture_output=True)
        except:
            raise Exception('An error occurred during the owner setting')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for plotting statistics about the controllers contained in '
                                                 'the hall of fame of multiple experiments.')
    parser.add_argument('exps_folders', metavar='exps_folders', type=str, nargs='+',
                        help='list of experiments folders containing hall_of_fame files and corresponding directories '
                             'with controllers files')
    parser.add_argument('--parent-folders-num', metavar='parent-folders-num', type=int, action='store',
                        default=1, help='number of parent folders to traverse before creating the out folder')
    parser.add_argument('--max-num-individuals', metavar='max-num-individuals', type=int, action='store',
                        default=101, help='maximum number of individuals to consider in Hall of Fames')
    parser.add_argument('--out-file', metavar='out-file', type=str, action='store',
                        default='hof_controllers_stats.pdf', help='output filename')
    parser.add_argument('--labels', metavar='labels', type=str, action='store', nargs='+',
                        default=None, help='charts labels - must match the number of experiments!')
    parser.add_argument('--owner', metavar='owner', type=str, action='store', nargs='+',
                        default=None, help='User and group for chown')

    args = parser.parse_args()
    main(args.exps_folders, args.max_num_individuals, args.parent_folders_num, args.out_file, args.labels, args.owner)
