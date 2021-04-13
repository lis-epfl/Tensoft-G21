#! /usr/bin/env python3

# ================================== #
#        VERIFICATION SCRIPT         #
#   Controls that generated robot    #
#     provides the same fitness      #
#        as the one reported         #
# ================================== #

import time
import argparse
import subprocess
from numpy import asarray, mean, isclose

SIMULATION_PATH = '../apps/robotSimulation'

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


def main(robot_file, noise_type, noise_level, sim_seed):
    modules_conf, fitness = load_configuration(robot_file)
    robot_string = convert_config(modules_conf)

    fitnesses = []

    print('Simulating...')
    for init_pos in range(3):
        exec_string = '{} {} {} {} {} {}'.format(SIMULATION_PATH,
                                                 noise_type,
                                                 noise_level,
                                                 init_pos,
                                                 robot_string,
                                                 sim_seed)
        start = time.time()
        cproc = subprocess.run(exec_string.split(' '), capture_output=True)
        print('Exec time:', time.time() - start)

        fitnesses.append(float(cproc.stdout.decode("utf-8").strip().split(' ')[-1]))

    current_fitness = min(fitnesses)
    if isclose(fitness, current_fitness):
        print('\nFitnesses correspons!')
    else:
        print('\nFitnesses diverges!')
    print('EA computed fitness: {} - Current Fitness {}'.format(fitness, current_fitness))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for checking that generated fitness is correct.')
    parser.add_argument('robot_file', metavar='robot_file', type=str,
                        help='the file where robot configuration is stored')
    parser.add_argument('noise_type', metavar='noise_type', type=int, default=1,
                        help='which type of noise has been employed (0: no noise; 1: gaussian; 2: uniform)')
    parser.add_argument('noise_level', metavar='noise_level', type=float, default=0.035,
                        help='the amount of noise introduced in the control signal')
    parser.add_argument('sim_seed', metavar='sim_seed', type=float, default=42,
                        help='simulation seed employed for the generation of the noise values')
    args = parser.parse_args()

    main(args.robot_file, args.noise_type, args.noise_level, args.sim_seed)