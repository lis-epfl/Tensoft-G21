import argparse
import json
import matplotlib
import os
import pickle
import re
import sys

from collections import Counter
from deap import base, creator
from matplotlib import pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import robot

from utils import parse_robot_string


# DEAP creator utils - they are required to load population pkl
creator.create('FitnessMax', base.Fitness, weights=(1.0,))
creator.create('Robot', robot.Robot, fitness=creator.FitnessMax)


def plot_actuation_stats(populations, results_dir):
    actuation_stats_dir = os.path.join(results_dir, 'actuation')
    os.makedirs(actuation_stats_dir, exist_ok=True)

    synchronizations = []
    axial_actuations = []

    for pop in populations:
        pop_synchronizations = []
        pop_axial_actuations = []
        for robot in pop:
            phases = [module['phase'] for module in robot]
            counter_phases = Counter(phases)
            connected_faces = [module['connectedFaces'] for module in robot]

            pop_synchronizations.append(
                counter_phases.most_common(1)[0][1] / len(phases)
            )
            pop_axial_actuations.append(
                (connected_faces.count(1) + connected_faces.count(5)) / len(connected_faces)
            )
        synchronizations.append(pop_synchronizations)
        axial_actuations.append(pop_axial_actuations)

    conf = {
        'data': synchronizations,
        'title': 'Modules synchronization across last generation',
        'x_tick_labels': ['Run {}'.format(i) for i in range(len(synchronizations))],
        'y_label': 'Synchronization ratio',
        'ratio': True,
        'out_file': os.path.join(actuation_stats_dir, 'synchronization.pdf')
    }
    plot_stats_boxplot(conf)

    conf = {
        'data': axial_actuations,
        'title': 'Axial actuation across last generation',
        'x_tick_labels': ['Run {}'.format(i) for i in range(len(axial_actuations))],
        'y_label': 'Axial actuation ratio',
        'ratio': True,
        'out_file': os.path.join(actuation_stats_dir, 'axial_actuation.pdf')
    }
    plot_stats_boxplot(conf)


def plot_stats_boxplot(conf):
    font = {'family': 'Source Sans Pro', 'size': 18, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    fig = plt.figure(figsize=(10, 8))
    ax = fig.gca()
    ax.boxplot(conf['data'])
    ax.set_title(conf['title'], fontweight='normal')
    if conf['ratio']:
        ax.set_ylim(-0.1, 1.1)
    ax.set_xticklabels(conf['x_tick_labels'])
    ax.set_ylabel(conf['y_label'])

    plt.savefig(conf['out_file'], bbox_inches='tight')
    plt.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for generating boxplots of '
                                                 'actuation stats for locomotion task.')
    parser.add_argument('res_dir', metavar='res_dir', type=str, nargs='?',
                        default='res_dir', help='the folder where results are stored')
    args = parser.parse_args()

    with open(os.path.join(args.res_dir, 'settings.json')) as settings_file:
        settings = json.load(settings_file)

    if settings['algorithm']['name'] in ['mu+lambda', 'vie']:
        pop_files = {int(re.split('[_.]', f)[-2]): os.path.join(args.res_dir, f)
                     for f in os.listdir(args.res_dir)
                     if re.match('pop_[0-9]+_sim_[0-9]+\.pkl', f)}

        last_pops = []
        for i in range(0, len(pop_files)):
            with open(pop_files[i], 'rb') as cpf:
                pop = pickle.load(cpf)

            last_pops.append([rb.get_modules_conf() for rb in pop])
    else:
        evo_files = {int(re.split('[_.]', f)[-2]): os.path.join(args.res_dir, f)
                     for f in os.listdir(args.res_dir)
                     if re.match('evo_[0-9]+_sim_[0-9]+\.json', f)}

        last_pops = []
        for i in range(0, len(evo_files)):
            with open(evo_files[i]) as evo_file:
                evo_file_obj = json.load(evo_file)

            last_gen = [parse_robot_string(robot['sim_string'])
                        for robot in evo_file_obj[-1]['population']]
            last_pops.append(last_gen)

    plot_actuation_stats(last_pops, args.res_dir)
