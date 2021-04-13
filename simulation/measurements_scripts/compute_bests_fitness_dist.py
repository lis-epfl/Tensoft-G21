#! /usr/bin/env python3

# ==================================== #
#      ROBOT FITNESS DISTRIBUTION      #
#  Compute box plot of robot fitness   #
#        for each robot feature        #
# ==================================== #

import os
import re
import time
import pickle
import argparse
import subprocess
import matplotlib
import multiprocessing
import matplotlib.pyplot as plt

from numpy import asarray, mean, max, ceil

SIMULATION_PATH = '../apps/robotSimulation'
NUM_TESTS = 10
STIFF_VALUES = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]
STIFF2ID = { str(b): a for a, b in enumerate(STIFF_VALUES) }


def load_configuration(robot_file):
    """ Read from file robot configuration
    generated for visualization

    :param robot_file: the file where the configuration is stored
    :return: a tuple containing a list of strings that represents robot modules
             and robot fitness
    """
    with open(robot_file) as in_file:
        modules = [line.strip().split(' ') for line in in_file]

    # last value is robot fitness
    return modules[:-1], float(modules[-1][-1])


def convert_config(robot_modules):
    """ Transform robot modules configuration from visualization
        format to inline format

    :param robot_modules: a list of strings that represents robot modules
    :return: a single string representing inline robot configuration
    """
    robot_string = ''

    for mod in robot_modules:
        data = asarray(mod)[list(range(1, 16, 2))]
        robot_string += '{} - {} - {} - {} - {} - {} - {} - {} -- '.format(*data)

    return robot_string


def simulate(input_conf):
    fit_res = []
    for _ in range(NUM_TESTS):
        for pos in range(3):
            exec_string = '{} {} {} {} {}'.format(SIMULATION_PATH,
                                                  input_conf[1],
                                                  input_conf[2],
                                                  pos,
                                                  input_conf[0])
            cproc = subprocess.run(exec_string.split(' '), capture_output=True)
            fit_res.append(float(cproc.stdout.decode("utf-8").strip().split(' ')[-1]))

    return fit_res


def fitness_box_plot(nmods, data, max_val, out_dir):
    font = {'family': 'Source Sans Pro', 'size': 14, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 200

    fig = plt.figure(figsize=(12, 12))
    ax = fig.gca()
    ax.set_title('Robot Fitness Distribution - # modules: {}'.format(nmods), size=24,
                 fontweight='light')
    ax.set_xlabel('Stiffness', labelpad=10)
    ax.set_ylabel('Fitness', labelpad=10)
    ax.set_ylim(0, ceil(max_val))

    bp = ax.boxplot(data, labels=STIFF_VALUES)

    for median in bp['medians']:
        median.set(linewidth=1.2)

    plt.savefig(os.path.join(out_dir, 'dist_robots_n_mods_{}.pdf'.format(nmods)),
                bbox_inches='tight')


def fitness_box_plot_2(data, max_val, out_dir):
    font = {'family': 'Source Sans Pro', 'size': 16, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    fig, axs = plt.subplots(2, 5, sharex='all', sharey='all', figsize=(24, 10))
    fig.suptitle('Robots Fitness Distribution', fontsize=28, fontweight='normal')

    for i, stiff in enumerate(STIFF_VALUES):
        axs[i//5][i%5].set_title('Stiffness: {}'.format(stiff))

        if i // 5 == 1:
            axs[i//5][i%5].set_xlabel('# Modules', labelpad=10, fontsize=16, fontweight='light')
        if i % 5 == 0:
            axs[i//5][i%5].set_ylabel('Fitness [cm]', labelpad=12, fontsize=16, fontweight='light')

        axs[i//5][i%5].set_ylim(0, ceil(max_val))

        bp = axs[i//5][i%5].boxplot(data[i], labels=list(range(2, 11)))

        for median in bp['medians']:
            median.set(linewidth=1.2)

    plt.subplots_adjust(wspace=0.1, hspace=0.18)
    plt.savefig(os.path.join(out_dir, 'global_fit_dist.pdf'), bbox_inches='tight')


def main(bests_folder, out_dir, noise_type, noise_level, comp_data_file=None):
    start_time = time.time()

    os.makedirs(out_dir, exist_ok=True)

    if comp_data_file is None:
        robot_configs_per_sim = [f for f in os.listdir(bests_folder) if re.match('robot.*.txt', f)]
        if len(robot_configs_per_sim) == 0:
            raise Exception('No best robot file found! Please select correct folder.')

        # load robot configs and split them according to the feature space and simulation
        robot_configs = {}
        for rc in robot_configs_per_sim:
            key = '_'.join(rc.split('_')[1:3])
            if key not in robot_configs:
                robot_configs[key] = []

            robot_configs[key].append(rc)

        # simulate each robot configuration and collect the results
        fitness_data = {}
        max_val = 0.0
        with multiprocessing.Pool() as pool:
            for B, rob_confs in robot_configs.items():
                print('Simulating robot:', B)

                to_evaluate = []
                for sim_conf in rob_confs:
                    modules_conf, _ = load_configuration(os.path.join(bests_folder, sim_conf))
                    to_evaluate.append((convert_config(modules_conf), noise_type, noise_level))

                # parallelize computation in case of multiple runs have been performed
                fitness_data[B] = pool.map(simulate, to_evaluate)
                max_val = max([max(fitness_data[B]), max_val])

        with open(os.path.join(out_dir, 'fitness_data.pickle'), 'wb') as p_file:
            pickle.dump(fitness_data, p_file)

        # store results for later analysis (where results are still split among the different simulation)
        with open(os.path.join(out_dir, 'tmp_res.csv'), 'w') as out_file:
            for B, rf in fitness_data.items():
                out_file.write(str(B) + ',' + ','.join([str(mean(sim_rf)) for sim_rf in rf]) + '\n')
    else:
        with open(comp_data_file, 'rb') as pck_file:
            fitness_data = pickle.load(pck_file)
        max_val = max([max(v) for k, v in fitness_data.items()])

    # generate data for each box plot (split them according their number of modules and stiffness)
    box_plot_data = []
    for i in range(9):
        box_plot_data.append([])
        for _ in range(10):
            box_plot_data[i].append([0])

    for B, r_fits in fitness_data.items():
        nmods, stiff = B.split('_')
        data_idx = int(nmods) - 2
        stiff_idx = STIFF2ID[stiff]
        box_plot_data[data_idx][stiff_idx] = [fit for sim_fit in r_fits for fit in sim_fit]

    print('Plotting Box-Plots...')
    for nmods, data in enumerate(box_plot_data):
        fitness_box_plot(nmods + 2, data, max_val, out_dir)

    box_plot_data_2 = []
    for i in range(10):
        box_plot_data_2.append([])
        for _ in range(9):
            box_plot_data_2[i].append([0])

    for B, r_fits in fitness_data.items():
        nmods, stiff = B.split('_')
        data_idx = int(nmods) - 2
        stiff_idx = int(STIFF2ID[stiff])
        box_plot_data_2[stiff_idx][data_idx] = [fit for sim_fit in r_fits for fit in sim_fit]
    print('Plotting global Box-Plots...')
    fitness_box_plot_2(box_plot_data_2, max_val, out_dir)

    end_time = int(time.time() - start_time)
    print('Exec time: {:02d}:{:02d}:{:02d}'.format(end_time // 3600,
                                                   (end_time % 3600 // 60),
                                                   end_time % 60))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for plotting the fitness distribution of best robots')
    parser.add_argument('bests_folder', metavar='bests_folder', type=str,
                        help='where best robot config files are stored')
    parser.add_argument('out_dir', metavar='out_dir', type=str,
                        help='where fitness box-plots should be saved')
    parser.add_argument('noise_type', metavar='noise_type', type=int, default=1, nargs='?',
                        help='which type of noise has been employed (0: no noise; 1: gaussian; 2: uniform)')
    parser.add_argument('noise_level', metavar='noise_level', type=float, default=0.035, nargs='?',
                        help='the amount of noise introduced in the control signal')
    parser.add_argument('computed_data', metavar='computed_data', type=str, nargs='?', default=None,
                        help='where previous computation has been stored')
    args = parser.parse_args()

    main(args.bests_folder, args.out_dir, args.noise_type, args.noise_level, args.computed_data)
