#! /usr/bin/env python3

import argparse
import json
import multiprocessing
import neat
import numpy as np
import os
import random
import re
import shutil
import time

from itertools import count

from deap import base, creator, tools

import robot

from goal_reaching.controller import Controller
from goal_reaching.robot_gr import RobotGR
from goal_reaching.statistics_reporter import StatisticsReporterGR
from goal_reaching.utils_gr import open_history_file, restore_history_file, open_evo_file, \
    restore_evo_file, restore_best_controller, log_gen_info

from coevolution.utils_coev import load_morph_checkpoint, get_last_c_id, run_coev, update_history_and_hof

from utils import print_header_coev


# DEAP creator utils - they need to be global in order to be used with multiprocessing
creator.create('FitnessMax', base.Fitness, weights=(1.0,))
creator.create('Robot', robot.Robot, fitness=creator.FitnessMax)

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    Global variables needed to interact with the NEAT library
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
# Controllers directory
controllers_dir = ""

# History file
history_file = None
# Controller evolution file
contr_evo_file = None
# Morphology evolution file
morph_evo_file = None

# Sim_settings
morph_sim_settings = None

# Morphologies to be evaluated
morphologies_to_eval = []
# Related fitnesses values
fitnesses = []

# Morphology - id map
all_morphologies = None

# Hall of Fame
hall_of_fame = None

# Neat population
p = None
# Neat seed
seed = None

# Simulations number control variables
num_sims = None
max_num_sims = None

# Multiprocessing pool
pool = None
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''


def load_morph_sim_settings(toolbox, config):
    global controllers_dir

    toolbox.register('robot', gen_robot, creator.Robot,
                     simulation_path=config['simulation_path'],
                     tracker_path=config['tracker_path'],
                     noise_type=config.get('noise_type', 1),
                     noise_level=config.get('noise_level', 0.035),
                     **config['robot'])
    toolbox.register('population', tools.initRepeat, list, toolbox.robot)

    toolbox.register('evaluate', evaluate_individuals)
    toolbox.register('mate', robot.crossover_robots)

    # distinguish between 2D and 3D maps (multi-stiffness experiments)
    if config['robot'].get('per_module_stiffness', False):
        toolbox.register('mutate', robot.mutate_robot_per_module)
    else:
        toolbox.register('mutate', robot.mutate_robot)

    toolbox.register('selectBest', tools.selBest)
    toolbox.register('select', tools.selRoulette)
    toolbox.register('fill', tools.selRandom)

    toolbox.register('map', sim_coev)

    # prepare output folders
    results_dir = config['result_dir']
    best_dir = os.path.join(results_dir, 'best')
    checkpoint_dir = os.path.join(results_dir, 'checkpoints')
    controllers_dir = os.path.join(results_dir, 'controllers')
    evo_dir = os.path.join(results_dir, 'evolution_info')
    morphologies_dir = os.path.join(results_dir, 'morphologies')
    # Note: the result_dir must be a proper path where to store the files, otherwise the execution will fail
    os.makedirs(results_dir, exist_ok=True)
    os.makedirs(best_dir, exist_ok=True)
    os.makedirs(checkpoint_dir, exist_ok=True)
    os.makedirs(controllers_dir, exist_ok=True)
    os.makedirs(evo_dir, exist_ok=True)
    os.makedirs(morphologies_dir, exist_ok=True)

    return results_dir, checkpoint_dir, evo_dir, morphologies_dir


def gen_robot(create, simulation_path, tracker_path, noise_type=0, noise_level=1.0,
              num_faces=1, max_num_modules=10, mutation_config=None,
              modules_conf=None, robot_tests=3, per_module_stiffness=False, sim_seed=42):
    """ Support function for generating DEAP based individuals, that are robot encodings.
    """
    return create(simulation_path, tracker_path, noise_type, noise_level,
                  num_faces, max_num_modules, mutation_config, modules_conf,
                  robot_tests, per_module_stiffness, sim_seed)


def sim_coev(eval_function, morphologies):
    return eval_function(morphologies)


def evaluate_individuals(morphologies):
    global morphologies_to_eval

    # reset morphologies_to_eval array
    morphologies_to_eval = []

    # populate morphologies_to_eval
    for m in morphologies:
        string_repr = m.string_input()

        if string_repr not in all_morphologies:
            m_id = max(all_morphologies.values()) + 1
            all_morphologies[string_repr] = m_id
        else:
            m_id = all_morphologies[string_repr]

        morphologies_to_eval.append((m_id, m))

    # run one generation of controller evolution
    p.run(eval_controllers, 1)

    return fitnesses


# Evaluates the controllers provided
def eval_controllers(genomes, config):
    global fitnesses, num_sims

    # reset global fitnesses array
    fitnesses = []

    morph_len = len(morphologies_to_eval)

    # determine which controllers need to be evaluated and shuffle their indices
    controllers_indices = [
        i for i in range(0, len(genomes))
        if morph_sim_settings['eval_all'] or genomes[i][1].fitness is None
    ]
    random.shuffle(controllers_indices)

    # detect invalid controllers and collect valid controllers paths
    invalid_controllers = []
    controllers_paths = []
    for contr_indx in controllers_indices:
        genome_id, genome = genomes[contr_indx]

        controller = Controller(genome_id, seed, genome, config)
        controller_path = controller.save(controllers_dir, check_file_existence=morph_sim_settings['eval_all'])

        if controller_path is None:
            invalid_controllers.append(contr_indx)
            genomes[contr_indx][1].fitness = 0.0
        else:
            controllers_paths.append(controller_path)

    # remove invalid controllers from list
    controllers_indices = [indx for indx in controllers_indices if indx not in invalid_controllers]
    contr_len = len(controllers_indices)

    # map morphologies to controllers
    morph_to_controller_map = {}
    for i in range(0, max(morph_len, contr_len)):
        if i % morph_len in morph_to_controller_map:
            morph_to_controller_map[i % morph_len] += [controllers_indices[i % contr_len]]
        else:
            morph_to_controller_map[i % morph_len] = [controllers_indices[i % contr_len]]

    # prepare input for multiprocessing
    pool_input = []
    for morph_indx in morph_to_controller_map.keys():
        for contr_indx in morph_to_controller_map[morph_indx]:
            pool_input.append(
                (
                    morphologies_to_eval[morph_indx],
                    controllers_paths[controllers_indices.index(contr_indx)],
                    random.randint(0, 1000000)
                )
            )

    # evaluate pairs
    fitness_values = pool.map(eval_pair, pool_input)

    # associate fitness to morphologies
    index = 0
    for morph_indx in morph_to_controller_map.keys():
        morph_fits = []
        for contr_indx in morph_to_controller_map[morph_indx]:
            morph_fits.append(fitness_values[index])
            if genomes[contr_indx][1].fitness is not None:
                genomes[contr_indx][1].fitness += [fitness_values[index]]
            else:
                genomes[contr_indx][1].fitness = [fitness_values[index]]
            index += 1
        dist_vals = [1.0 / f for f in morph_fits]
        avg_val = float(np.mean(dist_vals))
        max_val = max(dist_vals)
        fit_val = 1.0 / (avg_val + 0.5 * (max_val - avg_val))
        fitnesses.append((fit_val,))

    # add None values to reflect the total number of evaluations
    for i in range(0, contr_len-morph_len):
        fitnesses.append(None)

    # associate fitness to controllers
    for i in controllers_indices:
        dist_vals = [1.0 / f for f in genomes[i][1].fitness]
        avg_val = float(np.mean(dist_vals))
        max_val = max(dist_vals)
        fit_val = 1.0 / (avg_val + 0.5 * (max_val - avg_val))
        genomes[i][1].fitness = fit_val

    # update num_sims
    num_sims += len(fitnesses)

    # update history and HoF
    update_history_and_hof(history_file, hall_of_fame, morph_sim_settings['hof_size'], p.generation,
                           morph_to_controller_map, morphologies_to_eval, genomes, fitness_values)

    # record controllers gen information
    log_gen_info(contr_evo_file, num_sims, genomes, num_sims != len(fitnesses))


# returns the fitness of the pair provided
def eval_pair(inputs):
    morphology_id = inputs[0][0]
    morphology = inputs[0][1]
    controller_path = inputs[1]
    sim_seeds_seed = inputs[2]

    r_sim = RobotGR(morph_sim_settings['direction_path'],
                    morph_sim_settings['rotation_angle_path']
                        if 'rotation_angle_path' in morph_sim_settings else None,
                    morphology.simulation_path, morphology.tracker_path,
                    morph_sim_settings['sim_time'], morphology.noise_type, morphology.noise_level,
                    morph_sim_settings['target_dist_bearing'], 0,
                    morphology.num_faces, morphology.num_modules, morphology.robot_tests,
                    sim_seeds_seed, morphology.get_modules_conf())
    fitness = r_sim.simulate(controller_path, m_id=morphology_id)

    return fitness


# performs the closing operations
def end_simulation(checkpoint_dir, checkpointer, stats_reporter, results_dir, morphologies_dir,
                   start_time, checkpoint_used):
    # save checkpoint of last controller generation (if it has not been done yet)
    if not os.path.exists(os.path.join(checkpoint_dir, 'contr_cp_' + str(seed) + '_' + str(p.generation - 1))):
        checkpointer.save_checkpoint(p.config, p.population, p.species, p.generation - 1)

    # save checkpoint of last generation stats (if it has not been done yet)
    if not os.path.exists(os.path.join(checkpoint_dir, 'contr_stats_cp_' + str(seed) + '_' + str(p.generation - 1))):
        stats_reporter.save_checkpoint(p.generation - 1, stats_reporter.most_fit_genomes,
                                       stats_reporter.generation_statistics)

    history_file.close()

    contr_evo_file.write(']')
    contr_evo_file.close()

    morph_evo_file.write(']')
    morph_evo_file.close()

    # copy best controller into best directory
    best_dir = os.path.join(results_dir, 'best')
    best_genome_id = hall_of_fame[min(hall_of_fame.keys())][0][1] if len(hall_of_fame.keys()) > 0 else None
    if best_genome_id is not None and \
            not os.path.exists(os.path.join(best_dir, 'controller_{}_{}.txt'.format(seed, best_genome_id))):
        shutil.copy2(
            os.path.join(
                controllers_dir,
                'controller_{}_{}.txt'.format(seed, best_genome_id)),
            best_dir
        )

    with open(os.path.join(morphologies_dir, 'morphologies_{}.json'.format(seed)), 'w') as morph_file:
        morph_file.write(json.dumps(all_morphologies, indent=4))

    with open(os.path.join(best_dir, 'hall_of_fame_{}.json'.format(seed)), 'w') as hof_file:
        hof_file.write(json.dumps(hall_of_fame, sort_keys=True, indent=4))

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
    parser = argparse.ArgumentParser(description='Script for simulating the co-evolution of morphology and controller '
                                                 'of a modular soft robot for goal reaching or goal reaching after '
                                                 'squeezing task')
    parser.add_argument('morphology_simulation_settings', metavar='morphology_simulation_settings', type=str, nargs='?',
                        default='./experiments_gr/algorithm.json', help='file name of the configuration file' +
                                ' containing the parameters for the simulation.' +
                                ' By default it looks for the ./experiments_gr/algorithm.json file.')
    parser.add_argument('neat_settings', metavar='neat_settings', type=str, nargs='?',
                        default='./experiments_gr/neat-settings.txt',
                        help='file name of the file containing the NEAT configuration' +
                        ' parameters. By default it uses ./experiments_gr/neat-settings.txt file.')
    parser.add_argument('--use-checkpoint', dest='use_checkpoint', action='store_const',
                        const=True, help='Select whether to run the experiment ' +
                                         'from previous checkpoints (if exist).')

    args = parser.parse_args()
    toolbox = base.Toolbox()

    # load morphology and simulation settings
    with open(str(args.morphology_simulation_settings)) as morphology_simulation_settings:
        morph_sim_settings = json.load(morphology_simulation_settings)

    # parse settings
    results_dir, checkpoint_dir, evo_dir, morphologies_dir = load_morph_sim_settings(toolbox, morph_sim_settings)

    # save settings
    with open(os.path.join(results_dir, 'settings.json'), 'w') as conf_file:
        json.dump(morph_sim_settings, conf_file, indent=4)

    # print header
    print_header_coev(morph_sim_settings)

    # morphology evolution algorithm
    algorithm = morph_sim_settings.get('algorithm')

    # seeds
    seeds = morph_sim_settings['seeds']
    for seed_val in seeds:
        # set init_time variable
        start_time = time.time()

        # set global seed accordingly
        seed = seed_val

        # init simulations vars
        num_sims = 0
        max_num_sims = morph_sim_settings['max_num_sims']

        # instantiate NEAT checkpointer
        checkpoint_prefix = 'contr_cp_' + str(seed) + '_'
        checkpointer = neat.Checkpointer(generation_interval=morph_sim_settings['checkpoint_freq'],
                                         time_interval_seconds=86400,
                                         filename_prefix=os.path.join(checkpoint_dir, checkpoint_prefix))

        # instantiate NEAT stats reporter
        stats_checkpoint_prefix = 'contr_stats_cp_' + str(seed) + '_'
        stats_reporter = StatisticsReporterGR(generation_interval=morph_sim_settings['checkpoint_freq'],
                                              filename_prefix=os.path.join(checkpoint_dir, stats_checkpoint_prefix))

        # in case of ViE set checkpoint properties according to the population parameters
        if algorithm['name'] == 'vie':
            gen_rate = algorithm['pop_size'] / algorithm['num_mutants']

            checkpointer.generation_interval = gen_rate * morph_sim_settings['checkpoint_freq']
            checkpointer.last_generation_checkpoint = -gen_rate

            stats_reporter.generation_interval = gen_rate * morph_sim_settings['checkpoint_freq']
            stats_reporter.last_generation_checkpoint = -gen_rate

        # history filename
        history_filename = os.path.join(evo_dir, 'history_'+str(seed)+'.csv')

        # morphology related checkpoint files
        morph_evo_filename = os.path.join(evo_dir, 'morph_evo_{}.json'.format(seed))
        morphology_checkpoints = {int(f.split('_')[-2]): os.path.join(checkpoint_dir, f)
                                  for f in os.listdir(checkpoint_dir)
                                  if re.match('checkpoint_[0-9]+_0\.pkl', f)}

        # controller related checkpoint files
        contr_evo_filename = os.path.join(evo_dir, 'contr_evo_{}.json'.format(seed))
        controller_checkpoints = {int(f.split('_')[-1]): os.path.join(checkpoint_dir, f)
                                  for f in os.listdir(checkpoint_dir)
                                  if re.match(checkpoint_prefix+'[0-9]', f)}

        # morphology population, checkpoint data, all morphologies and hall of fame initialization
        pop = None
        alg_data = None
        all_morphologies = {'_': 0}
        hall_of_fame = {}

        # if use_checkpoint option is True and the checkpoint files exist, load the populations from them
        if args.use_checkpoint and \
                seed in morphology_checkpoints and \
                os.path.exists(morphology_checkpoints[seed]) and  \
                len(controller_checkpoints) != 0 and \
                os.path.exists(controller_checkpoints[max(controller_checkpoints.keys())]):
            checkpoint_used = True

            # get index of the last checkpoint
            last_checkpoint = max(controller_checkpoints.keys())

            # load controllers population and adjust some properties
            p = neat.Checkpointer.restore_checkpoint(os.path.join(checkpoint_dir,
                                                                  checkpoint_prefix+str(last_checkpoint)))
            p.generation += 1
            p.reproduction.genome_indexer = count(max(p.population.keys())+1)

            # set last gen checkpoint property
            checkpointer.last_generation_checkpoint = last_checkpoint

            # load stats checkpoint
            most_fit_genomes, generation_statistics = StatisticsReporterGR.restore_checkpoint(
                os.path.join(checkpoint_dir, stats_checkpoint_prefix+str(last_checkpoint)))

            # restore stats reporter properties
            stats_reporter.most_fit_genomes = most_fit_genomes
            stats_reporter.generation_statistics = generation_statistics
            stats_reporter.last_generation_checkpoint = last_checkpoint

            # restore history_file
            history_file, num_sims = restore_history_file(history_filename, last_checkpoint)

            # restore evo files
            morph_evo_file, _ = restore_evo_file(morph_evo_filename, last_checkpoint, num_sims)
            contr_evo_file, contr_evo_obj = restore_evo_file(contr_evo_filename, last_checkpoint, num_sims)

            # restore best individual in population
            p.best_genome = restore_best_controller(contr_evo_obj, controllers_dir, seed)

            # load morphology EA checkpoint
            pop, alg_data, all_morphologies, hall_of_fame = \
                load_morph_checkpoint(morphology_checkpoints[seed], algorithm, num_sims)

            if morph_sim_settings['eval_all']:
                # remove controller files in excess w.r.t. checkpoint
                last_controller_id = get_last_c_id(contr_evo_obj)

                del_c_files = [
                    os.path.join(controllers_dir, f)
                    for f in os.listdir(controllers_dir)
                    if re.match('controller_{}_[0-9]+\.txt'.format(seed), f) and
                       int(re.split('_|\.', f)[-2]) > last_controller_id
                ]
                for file in del_c_files:
                    os.remove(file)

        else:
            checkpoint_used = False

            # init random number generator
            random.seed(seed)

            # load NEAT configuration
            neat_config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                                      neat.DefaultSpeciesSet, neat.DefaultStagnation,
                                      args.neat_settings)

            # adapt NEAT configuration by replacing num_output value
            neat_config.genome_config.num_outputs = 2*morph_sim_settings['robot']['max_num_modules']
            neat_config.genome_config.output_keys = [i for i in range(neat_config.genome_config.num_outputs)]

            # save config file
            neat_config.save(os.path.join(results_dir, 'neat_settings.txt'))

            # create the population
            p = neat.Population(neat_config)

            # open history file
            history_file = open_history_file(history_filename, type=1)

            # open evo files
            morph_evo_file = open_evo_file(morph_evo_filename)
            contr_evo_file = open_evo_file(contr_evo_filename)

            if morph_sim_settings['eval_all']:
                # delete potential controller files related to this seed
                del_c_files = [
                    os.path.join(controllers_dir, f)
                    for f in os.listdir(controllers_dir)
                    if re.match('controller_{}_[0-9]+\.txt'.format(seed), f)
                ]
                for file in del_c_files:
                    os.remove(file)

        # add checkpointer
        p.add_reporter(checkpointer)

        # add stats reporter
        p.add_reporter(stats_reporter)

        # multiprocessing pool instantiation
        pool = multiprocessing.Pool(processes=24)

        run_coev(toolbox, morph_sim_settings, checkpoint_dir, morph_evo_file, pop, alg_data,
                 seed_val, all_morphologies, hall_of_fame, robot.MIN_NUM_MODULES)

        # perform the closing operations
        end_simulation(checkpoint_dir, checkpointer, stats_reporter, results_dir,
                       morphologies_dir, start_time, checkpoint_used)
