import argparse
import json
import matplotlib
import numpy as np
import pandas as pd
import os
import sys

from collections import Counter
from matplotlib import pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coevolution.collect_hof_contr_data import str_from_bearing
from utils import parse_robot_string
from visualization.plot_actuation_stats import plot_stats_boxplot


def plot_hof_actuation_stats(results_dir, settings_file, seed, hall_of_fame_file, morphologies_file, best_only=False):
    # open all required files
    with open(settings_file) as sf:
        settings = json.load(sf)

    with open(hall_of_fame_file) as hof_file:
        hall_of_fame = json.load(hof_file)

    with open(morphologies_file) as morph_file:
        morphologies = json.load(morph_file)
    inverse_morph_dict = {v: k for k, v in morphologies.items()}

    # create results directory
    hall_of_fame_stats_dir = os.path.join(results_dir, 'hall_of_fame_stats')
    actuation_stats_dir = os.path.join(hall_of_fame_stats_dir, 'actuation')
    os.makedirs(hall_of_fame_stats_dir, exist_ok=True)
    os.makedirs(actuation_stats_dir, exist_ok=True)

    sim_time = []
    distances = []
    bearing_labels = []

    diff_freqs_per_target = []
    diff_phases_per_target = []

    phase_deltas_data_per_target = []
    null_phase_delta_data = []

    # avg_synch_data_per_target = []
    # synch_data_per_target_per_time = []
    # null_synch_data = []

    # prepare arrays for data collection
    for dist, bearing in settings['target_dist_bearing']:
        distances.append(dist)
        bearing_labels.append(str_from_bearing(bearing))

        diff_freqs_per_target.append([])
        diff_phases_per_target.append([])

        phase_deltas_data_per_target.append([])

        # avg_synch_data_per_target.append([])
        # synch_data_per_target_per_time.append([])

    # read HoF simulation data and populates data structures
    hall_of_fame_contr_data_dir = os.path.join(
        hall_of_fame_stats_dir,
        'hall_of_fame_{}_contr_data'.format(seed)
    )
    for pair_list in hall_of_fame.values():
        for m_id, c_id in pair_list:
            # number of differences
            sim_time, diff_freqs, diff_phases = get_diff_freq_phases(
                hall_of_fame_contr_data_dir, m_id, c_id, seed, distances, bearing_labels
            )

            for i in range(0, len(distances)):
                diff_freqs_per_target[i].append(float(np.mean(diff_freqs[i])))
                diff_phases_per_target[i].append(float(np.mean(diff_phases[i])))

            # deltas
            sim_time, pair_deltas_data, null_phase_delta = get_pair_phase_deltas_data(
                hall_of_fame_contr_data_dir, m_id, c_id, seed, distances, bearing_labels
            )

            for i in range(0, len(distances)):
                phase_deltas_data_per_target[i].append(float(np.mean(pair_deltas_data[i])))
            null_phase_delta_data.append(null_phase_delta)

            # synch
            '''
            sim_time, pair_synch_data, null_synch = get_pair_synch_data(
                hall_of_fame_contr_data_dir, m_id, c_id, seed, distances, bearing_labels
            )
            
            for i in range(0, len(distances)):
                avg_synch_data_per_target[i].append(float(np.mean(pair_synch_data[i])))
                for j in range(0, len(pair_synch_data[i])):
                    if len(synch_data_per_target_per_time[i])-1 < j:
                        synch_data_per_target_per_time[i].append([pair_synch_data[i][j]])
                    else:
                        synch_data_per_target_per_time[i][j].append(pair_synch_data[i][j])
        
            null_synch_data.append(null_synch)
            '''

            if best_only:
                break
        if best_only:
            break

    # plot different frequencies
    conf = {
        'data': diff_freqs_per_target,
        'title': 'Different frequencies w.r.t. 0 per target{}'.format(' across Hall of Fame' if not best_only else ''),
        'x_tick_labels': bearing_labels,
        'y_label': '# of different frequencies',
        'ratio': False,
        'out_file': os.path.join(actuation_stats_dir,
                                 'different_frequencies_per_target_{}{}.pdf'.format(seed, '_best' if best_only else ''))
    }
    plot_stats_boxplot(conf)

    # plot different phases
    conf = {
        'data': diff_phases_per_target,
        'title': 'Different phases w.r.t. 0 per target{}'.format(' across Hall of Fame' if not best_only else ''),
        'x_tick_labels': bearing_labels,
        'y_label': '# of different phases',
        'ratio': False,
        'out_file': os.path.join(actuation_stats_dir,
                                 'different_phases_per_target_{}{}.pdf'.format(seed, '_best' if best_only else ''))
    }
    plot_stats_boxplot(conf)

    '''
    # plot avg synch ratio over time for each target
    for i in range(0, len(distances)):
        average_per_time = []
        std_per_time = []
        for j in range(0, len(synch_data_per_target_per_time[i])):
            average_per_time.append(float(np.mean(synch_data_per_target_per_time[i][j])))
            std_per_time.append(float(np.std(synch_data_per_target_per_time[i][j])))

        conf = {
            'data': [sim_time, average_per_time,
                     np.asarray(average_per_time) - np.asarray(std_per_time),
                     np.asarray(average_per_time) + np.asarray(std_per_time)],
            'title': 'Average modules synchronization in time{}'.format(' across Hall of Fame' if not best_only else ''),
            'y_label': 'Avg. synchronization ratio',
            'out_file': os.path.join(actuation_stats_dir,
                                     'synchronization_{}_{}{}.pdf'.format(seed, bearing_labels[i],
                                                                          '_best' if best_only else ''))
        }
        plot_synch_trend_over_time(conf)
    '''

    # insert data related to null input and generate box-plots
    phase_deltas_data_per_target.insert(round(len(distances) / 2), null_phase_delta_data)
    # avg_synch_data_per_target.insert(round(len(distances)/2), null_synch_data)
    bearing_labels.insert(round(len(distances)/2), '0')

    # plot different phases
    conf = {
        'data': phase_deltas_data_per_target,
        'title': 'Phase deltas sum per target{}'.format(' across Hall of Fame' if not best_only else ''),
        'x_tick_labels': bearing_labels,
        'y_label': 'Phase delta sum',
        'ratio': False,
        'out_file': os.path.join(actuation_stats_dir,
                                 'phase_deltas_sum_per_target_{}{}.pdf'.format(seed, '_best' if best_only else ''))
    }
    plot_stats_boxplot(conf)

    # plot synch data
    '''
    conf = {
        'data': avg_synch_data_per_target,
        'title': 'Average modules synchronization per target{}'.format(' across Hall of Fame' if not best_only else ''),
        'x_tick_labels': bearing_labels,
        'y_label': 'Avg. synchronization ratio',
        'out_file': os.path.join(actuation_stats_dir,
                                 'synchronization_per_target_{}{}.pdf'.format(seed, '_best' if best_only else ''))
    }
    plot_stats_boxplot(conf)
    '''

    # collect data on axial actuation and generate box-plot
    axial_actuations = []
    for pair_list in hall_of_fame.values():
        for m_id, c_id in pair_list:
            robot = parse_robot_string(inverse_morph_dict[m_id])
            connected_faces = [module['connectedFaces'] for module in robot]

            axial_actuations.append(
                (connected_faces.count(1) + connected_faces.count(5)) / len(connected_faces)
            )

            if best_only:
                break
        if best_only:
            break

    conf = {
        'data': [axial_actuations],
        'title': 'Axial actuation{}'.format(' across Hall of Fame' if not best_only else ''),
        'x_tick_labels': ['HoF_{}'.format(seed)],
        'y_label': 'Axial actuation ratio',
        'ratio': True,
        'out_file': os.path.join(actuation_stats_dir, 'axial_actuation_{}{}.pdf'.format(seed,
                                                                                        '_best' if best_only else ''))
    }

    plot_stats_boxplot(conf)


def get_diff_freq_phases(hof_data_dir, m_id, c_id, seed, distances, bearing_labels):
    sim_time = []
    pair_diff_freqs = []
    pair_diff_phases = []

    # TODO: take multiple tests per individual into consideration
    for dist, bearing_label in zip(distances, bearing_labels):
        target_diff_freqs = []
        target_diff_phases = []

        sim_data = pd.read_csv(
            os.path.join(hof_data_dir,
                         'controller_data_{}_{}_{}_{}_{}_{}.csv'.format(m_id, c_id, seed, dist, bearing_label, 0))
        )

        sim_time = sim_data.iloc[1:, 0].tolist()

        zero_freqs = sim_data.iloc[0, :].filter(regex='frequency').values.tolist()
        zero_phases = sim_data.iloc[0, :].filter(regex='phase').values.tolist()

        for i in range(1, 2):
            freqs = sim_data.iloc[i, :].filter(regex='frequency').values.tolist()
            phases = sim_data.iloc[i, :].filter(regex='phase').values.tolist()

            diff_freqs = 0
            diff_phases = 0
            for j in range(0, len(freqs)):
                if zero_freqs[j] != freqs[j]:
                    diff_freqs += 1
                if zero_phases[j] != phases[j]:
                    diff_phases += 1

            target_diff_freqs.append(diff_freqs)  # diff_freqs / len(freqs)
            target_diff_phases.append(diff_phases)  # diff_phases / len(phases)

        pair_diff_freqs.append(target_diff_freqs)
        pair_diff_phases.append(target_diff_phases)

    return sim_time, pair_diff_freqs, pair_diff_phases


def get_pair_phase_deltas_data(hof_data_dir, m_id, c_id, seed, distances, bearing_labels):
    sim_time = []
    pair_phase_deltas_data = []
    null_phase_deltas_data = None

    # TODO: take multiple tests per individual into consideration
    for dist, bearing_label in zip(distances, bearing_labels):
        target_deltas = []

        sim_data = pd.read_csv(
            os.path.join(hof_data_dir,
                         'controller_data_{}_{}_{}_{}_{}_{}.csv'.format(m_id, c_id, seed, dist, bearing_label, 0))
        )

        sim_time = sim_data.iloc[1:, 0].tolist()

        for i in range(0, 2):
            phases = sim_data.iloc[i, :].filter(regex='phase').values.tolist()

            deltas_sum = 0
            # deltas = []
            for j in range(0, len(phases)-1):
                deltas_sum += (phases[j]-phases[j+1])
                # deltas.append(phases[j]-phases[j+1])

            target_deltas.append(deltas_sum)
            # target_deltas.append(np.mean(deltas))

        pair_phase_deltas_data.append(target_deltas[1:])
        null_phase_deltas_data = target_deltas[0]

    return sim_time, pair_phase_deltas_data, null_phase_deltas_data


def get_pair_synch_data(hof_data_dir, m_id, c_id, seed, distances, bearing_labels):
    sim_time = []
    pair_synchs = []
    null_synch = None

    # TODO: take multiple tests per individual into consideration
    for dist, bearing_label in zip(distances, bearing_labels):
        target_synchs = []

        sim_data = pd.read_csv(
            os.path.join(hof_data_dir,
                         'controller_data_{}_{}_{}_{}_{}_{}.csv'.format(m_id, c_id, seed, dist, bearing_label, 0))
        )

        sim_time = sim_data.iloc[1:, 0].tolist()

        phases_data = sim_data.filter(regex='phase')
        for index, row in phases_data.iterrows():
            phases = row.values.tolist()
            counter_phases = Counter(phases)

            target_synchs.append(counter_phases.most_common(1)[0][1] / len(phases))

        pair_synchs.append(target_synchs[1:])
        null_synch = target_synchs[0]

    return sim_time, pair_synchs, null_synch


def plot_synch_trend_over_time(conf):
    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    fig = plt.figure(figsize=(12, 5))
    ax = fig.gca()

    ax.plot(conf['data'][0], conf['data'][1])
    ax.fill_between(conf['data'][0], conf['data'][2], conf['data'][3],
                    alpha=0.2, antialiased=True)

    ax.set_title(conf['title'], fontweight='normal')
    ax.set_ylim(-0.1, 1.1)
    ax.set_ylabel(conf['y_label'])

    plt.savefig(conf['out_file'], bbox_inches='tight')
    plt.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for generating plots of actuation stats about the individuals'
                                                 ' contained in the hall of fame (co-evolution & double map).')
    parser.add_argument('res_dir', metavar='res_dir', type=str, nargs='?',
                        default='res_dir', help='the folder where results are stored')
    parser.add_argument('--best-only', dest='best_only', action='store_const',
                        const=True, help='plot synch graphs only for the best individual')
    args = parser.parse_args()

    settings_file = os.path.join(args.res_dir, 'settings.json')
    with open(settings_file) as sf:
        settings = json.load(sf)

    for seed in settings['seeds']:
        hall_of_fame_file = os.path.join(args.res_dir, 'best', 'hall_of_fame_{}.json'.format(seed))
        morphologies_file = os.path.join(args.res_dir, 'morphologies', 'morphologies_{}.json'.format(seed))

        plot_hof_actuation_stats(args.res_dir, settings_file, seed,
                                 hall_of_fame_file, morphologies_file, args.best_only)
