#! /usr/bin/env python3

# ===================================== #
#   Test simulation time depending on   #
#   the different number of modules     #
#   that compose simulated robot        #
# ===================================== #

import os
import sys
sys.path.append('..')

import math
import json
import time
import robot
import pickle
import random
import argparse
import matplotlib
import numpy as np
import multiprocessing

# select matplotlib backend
matplotlib.use('pdf')
import matplotlib.pyplot as plt

from params_conf import MIN_NUM_MODULES, MAX_NUM_MODULES, N_MODULES
from timeit import timeit

# ======= CONFIG VARS ======= #
MUTATION_CONFIG = {
    "p_global_mut": 0.3,
    "p_local_mut": 0.6,
    "weights": {
        "global": [1, 1, 1],
        "local": [1, 1]
    }
}
MODULES_CONF_VAR = {
    "range_freq": [0.25, 0.5],
    "range_amp": [0.6],
    "range_phase": [0, 1.57, 3.14, 4.71],
    "stiff_table": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]
}
MODULES_CONF_FIXED = {
    "range_freq": [0.25, 0.5],
    "range_amp": [0.6],
    "range_phase": [0, 1.57, 3.14, 4.71],
    "stiff_table": [0.5]
}

SIMULATION_PATH = "../../simulation/apps/robotSimulation"

N_ROB_MODS = 10
N_TESTS = 10
N_POS = 3
# ============================================= #

def simulate(rob):
    pos_time = 0.0
    for pos in range(N_POS):
        exec_string = '{} {} {} {} {}'.format(rob.simulation_path,
                                              rob.noise_type,
                                              rob.noise_level,
                                              pos,
                                              rob.string_input())
        test_time = timeit(stmt='subprocess.getstatusoutput("{}")'.format(exec_string),
                           setup='import subprocess', number=N_TESTS)
        pos_time += (test_time / N_TESTS)
    return pos_time / N_POS


def plot_timings(data, out_name, is_fixed):
    font = {'family': 'Source Sans Pro', 'size': 16, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300
    colors = plt.cm.viridis(np.linspace(0, 1, len(N_MODULES)))

    fig = plt.figure(figsize=(12, 5))
    ax = fig.gca()

    x, y, std = list(zip(*[(k, v[0], v[1])
                           for k, v in sorted(data.items(), key=lambda r: float(r[0]))]))
    bar_plot = ax.bar(x, y, yerr=std, color=colors, label='Avg. Simulation Time')

    ax.set_title('Simulation Timings - Stiffness'
                 + '{} and Variable # modules'.format(0.5 if is_fixed else 'Variable'),
                 fontweight='normal', size=24)
    ax.set_xlabel('# Modules', fontweight='light')
    ax.set_ylabel('Avg. Simulation Time [s]', fontweight='light')

    for i, rect in enumerate(bar_plot):
        height = rect.get_height()
        ax.text(rect.get_x() + rect.get_width() / 2., height + 0.3,
                round(y[i], 2), ha='center', va='bottom', rotation=0)

    max_i = np.argmax(y)
    ax.set_ylim(0, math.ceil(y[max_i] + std[max_i])+1)
    plt.savefig(out_name, bbox_inches='tight')
    plt.close()


def main(out_folder, is_fixed=False):
    start_time = time.time()
    robot_types = {k: v for k, v in [(i, []) for i in N_MODULES]}
    exp_rand = random.Random(213)

    # generate random individuals for each number of modules
    num_generated = 0
    max_num2gen = N_ROB_MODS * (MAX_NUM_MODULES - MIN_NUM_MODULES + 1)

    mode = 'variable'
    modules_conf = MODULES_CONF_VAR
    if is_fixed:
        modules_conf = MODULES_CONF_FIXED
        mode = 'fixed'

        while num_generated < max_num2gen:
            new_rob = robot.Robot(SIMULATION_PATH, '',
                                  noise_type=1,
                                  noise_level=0.035,
                                  rnd=exp_rand,
                                  mutation_config=MUTATION_CONFIG,
                                  modules_conf=modules_conf)
            if len(robot_types[new_rob.num_modules]) < N_ROB_MODS:
                robot_types[new_rob.num_modules].append(new_rob)
                num_generated += 1
    else:
        while num_generated < max_num2gen:
            new_rob = robot.Robot(SIMULATION_PATH, '',
                                  noise_type=1,
                                  noise_level=0.035,
                                  rnd=exp_rand,
                                  mutation_config=MUTATION_CONFIG,
                                  modules_conf=modules_conf)
            if len(robot_types[new_rob.num_modules]) < N_ROB_MODS:
                # add N_ROB_MODS robots, each of them with a different stiffness
                stiffs = list(map(lambda r: r.stiffness, robot_types[new_rob.num_modules]))
                if new_rob.stiffness not in stiffs:
                    robot_types[new_rob.num_modules].append(new_rob)
                    num_generated += 1

    # ============================================= #
    # SIMULATIONS TIMING (# NUM MODULES DEPENDENTS) #
    robot_timings = {k: v for k, v in [(i, 0.0) for i in N_MODULES]}
    stat_robot_timings = {k: v for k, v in [(i, 0.0) for i in N_MODULES]}

    pool = multiprocessing.Pool()
    for n_mods in range(MIN_NUM_MODULES, MAX_NUM_MODULES + 1):
        print('Processing robots with {} modules...'.format(n_mods))

        total_sim_time = pool.map(simulate, robot_types[n_mods])

        print('-'*36)
        robot_timings[n_mods] = total_sim_time
        stat_robot_timings[n_mods] = (np.mean(total_sim_time), np.std(total_sim_time),
                                      np.median(total_sim_time))

    pool.close()

    # show and store the results
    print(stat_robot_timings)
    plot_timings(stat_robot_timings, os.path.join(out_folder, 'timings_{}.pdf'.format(mode)), is_fixed)

    with open(os.path.join(out_folder, 'timings_{}.txt'.format(mode)), 'w') as out_file:
        out_file.write(str(stat_robot_timings))

    with open(os.path.join(out_folder, 'timings_{}.json'.format(mode)), 'w') as json_file:
        json.dump(stat_robot_timings, json_file)

    with open(os.path.join(out_folder, 'timings_{}.pkl'.format(mode)), 'wb') as pickle_file:
        pickle.dump(robot_timings, pickle_file)

    end_time = int(time.time() - start_time)
    print('{:02d}:{:02d}:{:02d}'.format(end_time // 3600,
                                        (end_time % 3600 // 60),
                                        end_time % 60))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for control simulation timings')
    parser.add_argument('folder', metavar='folder', type=str, default='', help='in_out_folder')
    parser.add_argument('timings', metavar='timings', type=str, nargs='?',
                        default=None,
                        help='simulation timings file that have already been computed')
    parser.add_argument("--fixed", dest='is_fixed', action='store_const', default=False,
                        const=True, help="Select stiffness selection (variable or fixed).")

    args = parser.parse_args()
    if args.timings is None:
        main(args.folder, args.is_fixed)
    else:
        with open(os.path.join(args.folder, args.timings)) as in_file:
            data = json.load(in_file)
        mode = 'fixed' if args.is_fixed else 'variable'
        out_name = os.path.join(args.folder, 'timings_{}.pdf'.format(mode))
        plot_timings(data, out_name, args.is_fixed)
