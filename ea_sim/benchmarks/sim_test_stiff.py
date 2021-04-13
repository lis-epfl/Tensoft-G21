#! /usr/bin/env python3

# ========================================= #
#   Test simulation time depending on the   #
#   different stiffness value of modules    #
#   that compose simulated robot            #
# ========================================= #

import os
import sys
sys.path.append('..')

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

from params_conf import N_MODULES, STIFF_TABLE
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
MODULES_CONF = {
    "range_freq": [0.25, 0.5],
    "range_amp": [0.6],
    "range_phase": [0, 1.57, 3.14, 4.71],
    "stiff_table": []
}

SIMULATION_PATH = "../../simulation/apps/robotSimulation"

# fixed a specific number of modules to check stiffness value influence
N_MODULES=[5]

N_ROBS=3
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


def plot_timings(data, out_folder):
    font = {'family': 'Source Sans Pro', 'size': 16, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300
    colors = plt.cm.viridis(np.linspace(0, 1, len(STIFF_TABLE)))

    fig = plt.figure(figsize=(12, 5))
    ax = fig.gca()

    x, y, std = list(zip(*[(str(k), v[0], v[1])
                           for k, v in sorted(data.items(), key=lambda r: float(r[0]))]))
    bar_plot = ax.bar(x, y, yerr=std, color=colors, label='Avg. Simulation Time')

    ax.set_title('Simulation Timings - Variable Stiffnes with 5 modules',
                 fontweight='normal', size=24)
    ax.set_xlabel('Stiffness', fontweight='light')
    ax.set_ylabel('Avg. Simulation Time [s]', fontweight='light')

    for i, rect in enumerate(bar_plot):
        height = rect.get_height()
        ax.text(rect.get_x() + rect.get_width() / 2., height + 0.3,
                round(y[i], 2), ha='center', va='bottom', rotation=0)

    ax.set_ylim(0, max(y)+5)
    plt.savefig(os.path.join(out_folder, 'timings_stiff.pdf'), bbox_inches='tight')
    plt.close()


def main(out_folder):
    start_time = time.time()
    robot_types = {k: v for k, v in [(stiff, []) for stiff in STIFF_TABLE]}
    exp_rand = random.Random(213)

    for stiff in STIFF_TABLE:
        # select current stiffness value
        MODULES_CONF['stiff_table'] = [stiff]

        # add N_ROBS robots with n_mods modules for each stiffness value
        for n_mods in N_MODULES:
            for _ in range(N_ROBS):
                new_rob = robot.Robot(SIMULATION_PATH, '',
                                      noise_type=1,
                                      noise_level=0.035,
                                      rnd=exp_rand,
                                      mutation_config=MUTATION_CONFIG,
                                      modules_conf=MODULES_CONF)

                # generate a new robot until one with the desired number of modules is returned
                while new_rob.num_modules != n_mods:
                    new_rob = robot.Robot(SIMULATION_PATH, '',
                                          noise_type=1,
                                          noise_level=0.035,
                                          rnd=exp_rand,
                                          mutation_config=MUTATION_CONFIG,
                                          modules_conf=MODULES_CONF)

                robot_types[new_rob.stiffness].append(new_rob)

    # ============================================= #
    # SIMULATIONS TIMING (# NUM MODULES DEPENDENTS) #
    robot_timings = {k: v for k, v in [(stiff, 0.0) for stiff in STIFF_TABLE]}
    stat_robot_timings = {k: v for k, v in [(stiff, 0.0) for stiff in STIFF_TABLE]}

    pool = multiprocessing.Pool()
    for stiff in STIFF_TABLE:
        print('Processing robots with stiffness {}...'.format(stiff))

        total_sim_time = pool.map(simulate, robot_types[stiff])

        print('-'*39)
        robot_timings[stiff] = total_sim_time
        stat_robot_timings[stiff] = (np.mean(total_sim_time), np.std(total_sim_time),
                                     np.median(total_sim_time))

    pool.close()

    # show and store the results
    print(stat_robot_timings)
    plot_timings(stat_robot_timings, out_folder)

    with open(os.path.join(out_folder, 'timings_stiff.txt'), 'w') as out_file:
        out_file.write(str(stat_robot_timings))

    with open(os.path.join(out_folder, 'timings_stiff.json'), 'w') as json_file:
        json.dump(stat_robot_timings, json_file)

    with open(os.path.join(out_folder, 'timings_stiff.pkl'), 'wb') as pickle_file:
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
                        help='simulation timings json file that have already been computed')

    args = parser.parse_args()
    if args.timings is None:
        main(args.folder)
    else:
        with open(os.path.join(args.folder, args.timings)) as in_file:
            data = json.load(in_file)
        plot_timings(data, args.folder)
