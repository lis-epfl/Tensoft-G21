#! /usr/bin/env python3

# ============================== #
#   This script is employed to   #
#     compute (evolve) the Ks    #
#   parameters that are used in  #
#     the NN controller formula  #
# ============================== #

import os
import time
import random
import argparse

import subprocess
import numpy as np
import multiprocessing

import vie_measure # ViE minimization version with constraints

from deap import base, creator, tools

# ============ EVOLUTION CONFIGURATION ============ #

# length of the command input step
STEP_LEN = 10000
INIT_REC_0 = STEP_LEN*5
INIT_REC_180 = STEP_LEN*11 # skip init + 0 + transition values

# mutation configuration
MUT_PROB = 0.4

MIN_CONSTRAINS = [1.0]*5
MAX_CONSTRAINS = [2.0]*5

TARGET_COMPRESSION = 0.4 # stiffness: 0.1

SIM_SEED = 42
# ================================================= #

def evaluate_sol(solution):
    # prog <robot_conf> <K1> ... <K5> <noiseType> <noiseLevel>
    exec_string = '{} {} {} {} {}'.format('../apps/robotMeasurements',
                                          '../test_inputs/stiff_single/single_0.1.txt',
                                          str(solution),
                                          0,
                                          0,
                                          SIM_SEED)
    cproc = subprocess.run(exec_string.split(' '), capture_output=True)
    raw_data = cproc.stdout.decode("utf-8").strip().split('è,')[1:-1]

    data = np.asarray([[[float(d) for d in e.split(';')] for e in line.split(',')[1:]]
                       for line in raw_data])[:, 1:-1]

    # extract from the data only the initial compression values and the final ones
    initial_coms = np.median(data[list(range(INIT_REC_0-1, (INIT_REC_0+STEP_LEN*3)))], axis=0)    # command 0
    final_coms = np.median(data[list(range(INIT_REC_180-2, (INIT_REC_180+STEP_LEN*3)-2))], axis=0)  # command 180

    init_face_com_1 = np.mean(initial_coms[[1, 3, 4]], axis=0)
    init_face_com_5 = np.mean(initial_coms[[0, 2, 5]], axis=0)
    init_comp = np.linalg.norm(init_face_com_1 - init_face_com_5)

    fin_face_com_1 = np.mean(final_coms[[1, 3, 4]], axis=0)
    fin_face_com_5 = np.mean(final_coms[[0, 2, 5]], axis=0)
    fin_comp = np.linalg.norm(fin_face_com_1 - fin_face_com_5)

    # relative compression wrt to initial position
    current_compression = fin_comp/init_comp

    # consider invalid solutions the ones that compress the module more than the requirements
    return (current_compression, ) if current_compression >= TARGET_COMPRESSION else (1, )

def mutate_sol(solution):
    # gaussian mutation
    for i in range(len(solution.ks)):
        if random.random() < MUT_PROB:
            solution.ks[i] += np.random.normal(0, 0.12)

    return solution,


def cross(sol1, sol2):
    return sol1, sol2


class ModuleEvoConf:
    def __init__(self, min_constraints, max_constraints):
        self.min_constraints = min_constraints
        self.max_constraints = max_constraints
        self.ks = [min_constraints[i] + np.abs(np.random.normal(0, 0.4))
                     for i in range(len(min_constraints))]

    def __str__(self):
        return ' '.join(map(str, self.ks))

    def __repr__(self):
        return self.__str__()

    def __lt__(self, other):
        return self.fitness.values[0] < other.fitness.values[0]

    def __eq__(self, other):
        return (self.ks == other.ks
                and self.fitness.values[0] == other.fitness.values[0])

    def check_viability(self):
        for i in range(len(self.ks)):
            if self.ks[i] < self.min_constraints[i] or self.ks[i] > self.max_constraints[i]:
                return False

        return True

def gen_sol(create, min_constraints, max_constraints):
    """ Generate a random individual for the population. """
    return create(min_constraints, max_constraints)


creator.create('FitnessMin', base.Fitness, weights=(-1.0,))
creator.create('Configuration', ModuleEvoConf, fitness=creator.FitnessMin)


def main(tbox):
    # ======= SIMULATION CONFIGURATION ======= #
    max_num_sims = 12000
    pop_size = 32
    num_mutants = 32
    conv_rate = 0.05
    min_update = 0.001
    seeds = [42, 7240, 213, 96542, 10537996]

    # define output files and directories
    results_file = os.path.join('outputs', 'module_evo_results.csv')
    if not os.path.exists(os.path.dirname(results_file)):
        os.makedirs(os.path.dirname(results_file), exist_ok=True)
    # ======================================== #

    bests_history = []
    start_time = time.time()

    for seed in seeds:
        random.seed(seed)
        np.random.seed(seed)
        pop = tbox.population(pop_size)

        print('Start ViE execution...')
        [pop, bests] = vie_measure.run(pop, tbox, max_num_sims,
                                       conv_rate, num_mutants, min_update)
        print('ViE execution terminated!\n')

        bests_history.append(bests)

        for bh in bests_history:
            print('seed: {}'.format(seed))
            for b in sorted(bh):
                print('\t', b.fitness.values[0], '\t', b)

    with open(results_file, 'w') as out_file:
        out_file.write('fitness,K1,K4,K6,K7,K10\n')
        for bh in bests_history:
            for b in sorted(bh):
                out_file.write(str(b.fitness.values[0]) + ',' + ','.join(str(b).split(' ')) + '\n')

    end_time = int(time.time() - start_time)
    # record how much time it needed for current experiment
    elap_time = '{:02d}:{:02d}:{:02d}'.format(end_time // 3600,
                                              (end_time % 3600 // 60),
                                              end_time % 60)

    print('Computation time: {}'.format(elap_time))


if __name__ == '__main__':
    assert 0.0 <= TARGET_COMPRESSION <= 1.0, 'TARGET_COMPRESSION must be within the interval [0, 1].'

    parser = argparse.ArgumentParser(description='Script for evolving module parameters (Ks)')
    args = parser.parse_args()

    # prepare the structure of the EA algorithm
    toolbox = base.Toolbox()
    toolbox.register('conf', gen_sol, creator.Configuration, MIN_CONSTRAINS, MAX_CONSTRAINS)
    toolbox.register('population', tools.initRepeat, list, toolbox.conf)

    toolbox.register('evaluate', evaluate_sol)
    toolbox.register('mutate', mutate_sol)
    toolbox.register('mate', cross)

    toolbox.register('selectBest', tools.selBest)
    toolbox.register('select', tools.selRoulette)
    toolbox.register('fill', tools.selRandom)

    pool = multiprocessing.Pool()
    toolbox.register('map', pool.map)

    # execute the evolutionary simulation
    main(toolbox)
    pool.close()
