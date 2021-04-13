import argparse
import neat
import os
import subprocess
import re
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from goal_reaching.utils_gr import parse_controller
from visualization_adv_tasks.plot_hof_controllers_stats import compute_rates
from visualization_adv_tasks.plot_multi_hof_controllers_stats import plot_boxplots
# from visualization.plot_actuation_stats import plot_stats_boxplot
# from scipy.stats import ranksums


def load_contr_stats(i, controller_file, neat_settings):
    print('\t\tRun #{}'.format(i))

    neat_config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                              neat.DefaultSpeciesSet, neat.DefaultStagnation,
                              neat_settings)

    genome, _, _ = parse_controller(controller_file, 0)
    controller_nn = neat.nn.FeedForwardNetwork.create(genome, neat_config)

    i_o_rates, i_h_rates, h_h_rates, h_o_rates, o_h_rates, o_o_rates = \
        compute_rates([(genome, controller_nn, int(neat_config.genome_config.num_outputs/2))])

    return i_o_rates, i_h_rates, h_h_rates, h_o_rates, o_h_rates, o_o_rates


def load_experiment(i, folder):
    print('\tExperiment #{}'.format(i))

    # load single run bests stats
    best_files = [(os.path.join(folder, f), os.path.join(folder, 'neat_settings.txt'))
                  for f in os.listdir(folder)
                  if re.match('controller.*\.txt', f)]
    best_files.sort(key=lambda element: int(re.split('_|\.', element[0])[-2]))

    i_o_rates_lists, i_h_rates_lists, h_h_rates_lists, h_o_rates_lists, o_h_rates_lists, o_o_rates_lists = \
        list(zip(*[load_contr_stats(j, cf, nsf)
                   for j, (cf, nsf) in enumerate(best_files)]))

    i_o_rates = [rate for rates in i_o_rates_lists for rate in rates]
    i_h_rates = [rate for rates in i_h_rates_lists for rate in rates]
    h_h_rates = [rate for rates in h_h_rates_lists for rate in rates]
    h_o_rates = [rate for rates in h_o_rates_lists for rate in rates]
    o_h_rates = [rate for rates in o_h_rates_lists for rate in rates]
    o_o_rates = [rate for rates in o_o_rates_lists for rate in rates]

    return [i_o_rates, i_h_rates, h_h_rates, h_o_rates, o_h_rates, o_o_rates]


def main(exps_folders, parent_folders_num, out_file, labels, owner):
    print('Loading experiments...')
    plot_data = [load_experiment(i + 1, f)
                 for i, f in enumerate(exps_folders)]

    if labels is not None:
        if len(labels) != len(exps_folders):
            raise Exception('The number of provided labels'
                            + 'is different from the number'
                            + f'of experiments: {len(labels)}!={len(exps_folders)}')
    else:
        labels = [os.path.dirname(exp_folder).rsplit('/')[-1] for exp_folder in exps_folders]

    parent_folder = os.path.normpath(exps_folders[0])
    for i in range(0, parent_folders_num):
        parent_folder = os.path.dirname(parent_folder)
    out_folder = os.path.join(parent_folder, 'evo_plots')
    os.makedirs(out_folder, exist_ok=True)

    print('Plotting results...')
    conf = {
        'data': plot_data,
        'title': 'Connections usage in best controller across runs',
        'x_tick_labels': [
            'I-O', 'I-H', 'H-H', 'H-O', 'O-H', 'O-O'
        ],
        'x_labels': labels,
        'y_label': 'Conn. usage ratio',
        'out_file': os.path.join(out_folder, out_file)
    }
    plot_boxplots(conf)

    '''
    best_fits = [
        [29.468108125, 28.084806874999998, 29.222787499999995, 24.83102875, 25.646933125],
        [23.476965, 35.31125, 29.381075624999998, 27.094949375, 27.262569375]
    ]
    boxplots_conf = {
        'data': best_fits,
        'title': 'Distribution across runs of best overall fitness',
        'x_tick_labels': ['Exp 1', 'Exp 2'],
        'y_label': 'Fitness [cm]',
        'ratio': False,
        'out_file': os.path.join(out_folder, 'best_fitness.pdf')
    }
    plot_stats_boxplot(boxplots_conf)
    print(ranksums(best_fits[0], best_fits[1]))
    '''

    if owner is not None and len(owner) == 2:
        try:
            exec_string = 'chown -R {}:{} {}'.format(owner[0], owner[1], out_folder)
            c_proc = subprocess.run(exec_string.split(' '), capture_output=True)
        except:
            raise Exception('An error occurred during the owner setting')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for plotting statistics about the best controllers  '
                                                 'evolved in multiple experiments (controller evolution only).')
    parser.add_argument('exps_folders', metavar='exps_folders', type=str, nargs='+',
                        help='list of controller evolution experiments folders containing best files')
    parser.add_argument('--parent-folders-num', metavar='parent-folders-num', type=int, action='store',
                        default=1, help='number of parent folders to traverse before creating the out folder')
    parser.add_argument('--out-file', metavar='out-file', type=str, action='store',
                        default='best_controllers_stats.pdf', help='output filename')
    parser.add_argument('--labels', metavar='labels', type=str, action='store', nargs='+',
                        default=None, help='charts labels - must match the number of experiments!')
    parser.add_argument('--owner', metavar='owner', type=str, action='store', nargs='+',
                        default=None, help='User and group for chown')

    args = parser.parse_args()
    main(args.exps_folders, args.parent_folders_num, args.out_file, args.labels, args.owner)
