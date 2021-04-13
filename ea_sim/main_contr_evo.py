#! /usr/bin/env python3

import argparse
import json
import math
import multiprocessing
import neat
import os
import random
import re
import subprocess
import time
import shutil

from itertools import count
from tqdm import tqdm
from utils import print_header_contr_evo

from goal_reaching.controller import Controller
from goal_reaching.robot_gr import RobotGR
from goal_reaching.statistics_reporter import StatisticsReporterGR
from goal_reaching.utils_gr import open_history_file, restore_history_file, open_evo_file, restore_evo_file, \
    restore_best_controller, log_gen_info, update_history


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    Global variables needed to interact with the NEAT library
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
# Checkpoint directory
checkpoint_dir = None
# Controllers directory
controllers_dir = None
# Results directory
results_dir = None

# History file
history_file = None
# Controllers evolution file
contr_evo_file = None

# Robot configuration
robot_config = None
# Neat population
p = None
# Neat seed
seed = None

# Checkpointer instance
checkpointer = None
# Checkpoint usage
checkpoint_used = None

# Stats reporter instance
stats_reporter = None

# Simulations number control variables
num_sims = None
max_num_sims = None
sim_end = None

# Multiprocessing pool
pool = None

# Progress bar
pbar = None

# Start time
start_time = None
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''


# Evaluates the controllers provided
def eval_controllers(genomes, config):
    global num_sims, sim_end

    if num_sims < max_num_sims:
        # extract indices of controllers to evaluate and populate pool input
        controllers_indices = []
        pool_input = []

        for i in range(0, len(genomes)):
            if genomes[i][1].fitness is None:
                controller = Controller(genomes[i][0], seed, genomes[i][1], config)
                controller_path = controller.save(controllers_dir)

                if controller_path is not None:
                    controllers_indices.append(i)
                    pool_input.append([
                        controller_path,
                        random.randint(0, 1000000)
                    ])
                else:
                    genomes[i][1].fitness = 0.0

        # evaluate controllers
        fitnesses = pool.map(eval_controller, pool_input)
        num_evals = len(fitnesses)

        for i in range(0, num_evals):
            genomes[controllers_indices[i]][1].fitness = fitnesses[i]

        pbar.update(min(num_evals, max_num_sims - num_sims))
        num_sims += num_evals

        # update history
        update_history(history_file, p, genomes, controllers_indices)

        # log generation info
        log_gen_info(contr_evo_file, num_sims, genomes, num_sims != num_evals)
    else:
        if not sim_end:
            end_simulation(p.best_genome)
            p.remove_reporter(checkpointer)
            p.remove_reporter(stats_reporter)
            sim_end = True
        for c_id, genome in genomes:
            if genome.fitness is None:
                genome.fitness = 0.0


# returns the fitness of the controller provided
def eval_controller(inputs):
    controller_path = inputs[0]
    sim_seeds_seed = inputs[1]

    robot_sim = RobotGR(robot_config['direction_path'],
                        robot_config['rotation_angle_path']
                            if 'rotation_angle_path' in robot_config else None,
                        robot_config['simulation_path'], robot_config['tracker_path'],
                        robot_config['sim_time'], robot_config.get('noise_type', 1),
                        robot_config.get('noise_level', 0.035), robot_config['target_dist_bearing'],
                        robot_config['robot']['id'], robot_config['robot']['num_faces'],
                        robot_config['robot']['num_modules'], robot_config['robot']['robot_tests'],
                        sim_seeds_seed, robot_config['robot']['modules_conf'])
    fitness = robot_sim.simulate(controller_path)

    return fitness


# performs the closing operations
def end_simulation(best_genome):
    pbar.close()

    # save checkpoint of last generation (if it has not been done yet)
    if not os.path.exists(os.path.join(checkpoint_dir, 'contr_cp_' + str(seed) + '_' + str(p.generation - 1))):
        checkpointer.save_checkpoint(p.config, p.population, p.species, p.generation - 1)

    # save checkpoint of last generation stats (if it has not been done yet)
    if not os.path.exists(
            os.path.join(checkpoint_dir, 'contr_stats_cp_' + str(seed) + '_' + str(p.generation - 1))):
        stats_reporter.save_checkpoint(p.generation - 1, stats_reporter.most_fit_genomes,
                                       stats_reporter.generation_statistics)

    history_file.close()

    contr_evo_file.write(']')
    contr_evo_file.close()

    # copy best controller into best directory
    best_dir = os.path.join(results_dir, 'best')
    if best_genome is not None:
        shutil.copy2(os.path.join(controllers_dir,
                                  'controller_' + str(seed) + '_' + str(best_genome.key) + '.txt'), best_dir)

    # compute elapsed time
    end_time = int(time.time() - start_time)
    elap_time = '{:02d}:{:02d}:{:02d}'.format(end_time // 3600,
                                              (end_time % 3600 // 60),
                                              end_time % 60)

    # save elapsed time
    print('Total computation time: {}'.format(elap_time))
    if elap_time != '00:00:00':
        mode = 'a' if checkpoint_used else 'w+'
        with open(os.path.join(results_dir, 'elapsed_time_sim_{}.txt'.format(seed)), mode) as time_file:
            time_file.write(elap_time+'\n')

    pool.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for simulating via EA the evolution of the controller '
                                                 'of a modular soft robot for goal reaching or squeezing task')
    parser.add_argument('robot', metavar='robot', type=str, nargs='?',
                        default='./experiments_gr/robot.json', help='file name of the file' +
                        ' containing the robot configuration.' +
                        ' By default it looks for the ./experiments_gr/robot.json file.')
    parser.add_argument('neat_settings', metavar='neat_settings', type=str, nargs='?',
                        default='./experiments_gr/neat-settings.txt',
                        help='file name of the file containing the NEAT configuration' +
                        ' parameters. By default it uses ./experiments_gr/neat-settings.txt file.')
    parser.add_argument('--use-checkpoint', dest='use_checkpoint', action='store_const',
                        const=True, help='Select whether to run the experiment ' +
                                         'from previous checkpoints (if exist).')

    args = parser.parse_args()

    # load robot configuration
    with open(str(args.robot)) as robot_conf_file:
        robot_config = json.load(robot_conf_file)

    # print simulation header
    print_header_contr_evo(robot_config)

    # prepare output folders
    results_dir = robot_config['result_dir']
    best_dir = os.path.join(results_dir, 'best')
    checkpoint_dir = os.path.join(results_dir, 'checkpoints')
    controllers_dir = os.path.join(results_dir, 'controllers')
    evo_dir = os.path.join(results_dir, 'evolution_info')
    # Note: the result_dir must be a proper path where to store the files, otherwise the execution will fail
    os.makedirs(results_dir, exist_ok=True)
    os.makedirs(best_dir, exist_ok=True)
    os.makedirs(checkpoint_dir, exist_ok=True)
    os.makedirs(controllers_dir, exist_ok=True)
    os.makedirs(evo_dir, exist_ok=True)

    # save settings
    with open(os.path.join(results_dir, 'settings.json'), 'w') as conf_file:
        json.dump(robot_config, conf_file, indent=4)

    # store robot information to be used as input for simulation and visualization
    robot = RobotGR(None, None, robot_config['simulation_path'], None, robot_config['sim_time'],
                    robot_config.get('noise_type', 1), robot_config.get('noise_level', 0.035),
                    None, None, robot_config['robot']['num_faces'], robot_config['robot']['num_modules'],
                    None, None, robot_config['robot']['modules_conf'])
    robot.write_input_sim(os.path.join(results_dir, 'robot_sim.txt'))
    robot.write_input_vis(os.path.join(results_dir, 'robot_vis.txt'))

    # neat seeds
    seeds = robot_config['seeds']

    for seed_val in seeds:
        # set init_time variable
        start_time = time.time()

        # set global seed accordingly
        seed = seed_val

        # init simulations control vars
        num_sims = 0
        max_num_sims = robot_config['max_num_sims']
        sim_end = False

        # instantiate checkpointer
        checkpoint_prefix = 'contr_cp_' + str(seed) + '_'
        checkpointer = neat.Checkpointer(generation_interval=robot_config['checkpoint_freq'],
                                         time_interval_seconds=86400,
                                         filename_prefix=os.path.join(checkpoint_dir, checkpoint_prefix))

        # instantiate NEAT stats reporter
        stats_checkpoint_prefix = 'contr_stats_cp_' + str(seed) + '_'
        stats_reporter = StatisticsReporterGR(generation_interval=robot_config['checkpoint_freq'],
                                              filename_prefix=os.path.join(checkpoint_dir, stats_checkpoint_prefix))

        # history filename
        history_filename = os.path.join(evo_dir, 'history_' + str(seed) + '.csv')

        # evolution info and checkpoint files
        contr_evo_filename = os.path.join(evo_dir, 'contr_evo_{}.json'.format(seed))
        controller_checkpoints = {int(f.split('_')[-1]): os.path.join(checkpoint_dir, f)
                                  for f in os.listdir(checkpoint_dir)
                                  if re.match(checkpoint_prefix + '[0-9]', f)}

        # if use_checkpoint option is True and a checkpoint file exists, load the population from it
        if args.use_checkpoint and \
                len(controller_checkpoints) != 0 and \
                os.path.exists(controller_checkpoints[max(controller_checkpoints.keys())]):
            checkpoint_used = True

            # get index of the last checkpoint
            last_checkpoint = max(controller_checkpoints.keys())

            # load population
            p = neat.Checkpointer.restore_checkpoint(os.path.join(checkpoint_dir,
                                                                  checkpoint_prefix+str(last_checkpoint)))
            p.generation += 1

            p.reproduction.genome_indexer = count(max(p.population.keys()) + 1)

            # set last gen checkpoint property
            checkpointer.last_generation_checkpoint = last_checkpoint

            # load stats checkpoint
            most_fit_genomes, generation_statistics = StatisticsReporterGR.restore_checkpoint(
                os.path.join(checkpoint_dir, stats_checkpoint_prefix + str(last_checkpoint)))

            # restore stats reporter properties
            stats_reporter.most_fit_genomes = most_fit_genomes
            stats_reporter.generation_statistics = generation_statistics
            stats_reporter.last_generation_checkpoint = last_checkpoint

            # restore history file
            history_file, num_sims = restore_history_file(history_filename, last_checkpoint)

            # restore controller evolution file
            contr_evo_file, contr_evo_obj = restore_evo_file(contr_evo_filename, last_checkpoint, num_sims)

            # restore best individual in population
            p.best_genome = restore_best_controller(contr_evo_obj, controllers_dir, seed)
        else:
            checkpoint_used = False

            # init random number generator
            random.seed(seed)

            # load NEAT configuration
            neat_config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                                      neat.DefaultSpeciesSet, neat.DefaultStagnation,
                                      args.neat_settings)

            # adapt NEAT configuration by replacing num_output value
            neat_config.genome_config.num_outputs = 2*robot_config['robot']['num_modules']
            neat_config.genome_config.output_keys = [i for i in range(neat_config.genome_config.num_outputs)]

            # save config file
            neat_config.save(os.path.join(results_dir, 'neat_settings.txt'))

            # create the population
            p = neat.Population(neat_config)

            # open history file
            history_file = open_history_file(history_filename)

            # open controllers evolution file
            contr_evo_file = open_evo_file(contr_evo_filename)

        # add checkpointer
        p.add_reporter(checkpointer)

        # add stats reporter
        p.add_reporter(stats_reporter)

        # multiprocessing pool instantiation
        pool = multiprocessing.Pool(processes=24)

        # create progress bar
        pbar = tqdm(total=max_num_sims, postfix={})
        pbar.update(min(num_sims, max_num_sims))

        # run NEAT for the remaining generations
        best = p.run(eval_controllers, math.ceil(max_num_sims / p.config.pop_size) - p.generation)
        while num_sims < max_num_sims:
            best = p.run(eval_controllers, 1)

        # perform closing operations
        if not sim_end:
            end_simulation(best)
