#! /usr/bin/env python3

import argparse
import json
import multiprocessing
import neat
import os
import random
import re
import shutil
import time

from deap import base, creator, tools
from itertools import count

import robot

from goal_reaching.utils_gr import open_history_file, restore_history_file, open_evo_file, restore_evo_file
from double_map.nn_genome_operators import NNGenomeOperators
from double_map import map_elites_double_map
from double_map.utils_double_map import load_checkpoint, evaluate_individual
from utils import print_header_double_map

# DEAP creator utils - they need to be global in order to be used with multiprocessing
creator.create('FitnessMax', base.Fitness, weights=(1.0,))
creator.create('Robot', robot.Robot, fitness=creator.FitnessMax)


def load_evo_settings(toolbox, pool, evo_settings):
    # prepare output folders
    results_dir = evo_settings['result_dir']
    best_dir = os.path.join(results_dir, 'best')
    checkpoint_dir = os.path.join(results_dir, 'checkpoints')
    controllers_dir = os.path.join(results_dir, 'controllers')
    evo_dir = os.path.join(results_dir, 'evolution_info')
    morphologies_dir = os.path.join(results_dir, 'morphologies')
    tmp_contr_data_dir = os.path.join(results_dir, 'tmp_contr_data')
    # Note: the result_dir must be a proper path where to store the files, otherwise the execution will fail
    os.makedirs(results_dir, exist_ok=True)
    os.makedirs(best_dir, exist_ok=True)
    os.makedirs(checkpoint_dir, exist_ok=True)
    os.makedirs(controllers_dir, exist_ok=True)
    os.makedirs(evo_dir, exist_ok=True)
    os.makedirs(morphologies_dir, exist_ok=True)
    os.makedirs(tmp_contr_data_dir, exist_ok=True)

    # configure morphology evolution
    toolbox.register('entity', gen_robot, creator.Robot,
                     simulation_path=evo_settings['simulation_path'],
                     tracker_path=evo_settings['tracker_path'],
                     noise_type=evo_settings.get('noise_type', 1),
                     noise_level=evo_settings.get('noise_level', 0.035),
                     **evo_settings['robot'])
    toolbox.register('entity_population', tools.initRepeat, list, toolbox.entity)

    toolbox.register('mate_entity', robot.crossover_robots)
    # distinguish between 2D and 3D maps (multi-stiffness experiments)
    if evo_settings['robot'].get('per_module_stiffness', False):
        toolbox.register('mutate_entity', robot.mutate_robot_per_module)
    else:
        toolbox.register('mutate_entity', robot.mutate_robot)

    # configure simulation functions
    toolbox.register('evaluate', evaluate_individual)
    toolbox.register('map', pool.map)

    return results_dir, checkpoint_dir, controllers_dir, evo_dir, morphologies_dir


def gen_robot(create, simulation_path, tracker_path, noise_type=0, noise_level=1.0,
              num_faces=1, max_num_modules=10, mutation_config=None,
              modules_conf=None, robot_tests=3, per_module_stiffness=False, sim_seed=42):
    """ Support function for generating DEAP based individuals, that are robot encodings.
    """
    return create(simulation_path, tracker_path, noise_type, noise_level,
                  num_faces, max_num_modules, mutation_config, modules_conf,
                  robot_tests, per_module_stiffness, sim_seed)


def init_contr_genetic_operators(toolbox, evo_settings, neat_settings):
    # load NEAT configuration
    neat_config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                              neat.DefaultSpeciesSet, neat.DefaultStagnation,
                              neat_settings)

    # adapt NEAT configuration by replacing num_output value
    neat_config.genome_config.num_outputs = 2 * evo_settings['robot']['max_num_modules']
    neat_config.genome_config.output_keys = [i for i in range(neat_config.genome_config.num_outputs)]

    # configure controller evolution
    nn_genome_operators = NNGenomeOperators(neat_config)
    toolbox.register('last_nn_ids', nn_genome_operators.get_last_ids)

    toolbox.register('nn_genome', nn_genome_operators.new_genome)
    toolbox.register('nn_genome_population', tools.initRepeat, list, toolbox.nn_genome)

    toolbox.register('mate_nn_genome', nn_genome_operators.crossover)
    toolbox.register('mutate_nn_genome', nn_genome_operators.mutate)

    return nn_genome_operators


def run_evo(tbox, settings, checkpoint_dir, morph_map_evo_file, contr_map_evo_file,
            history_file, alg_data, seed, morphologies, hof, min_num_modules, neat_config):
    alg = settings.get('algorithm')
    time_no_update = alg.get('time_no_update', 10000)
    num_init_sols = alg.get('num_init_sols', 1200)
    batch_size = alg.get('batch_size', 24)

    if alg_data is None:
        init_rob = tbox.entity_population(num_init_sols)
        init_contr = tbox.nn_genome_population(num_init_sols)
        init_sols = list(zip(init_rob, init_contr))
    else:
        init_sols = [None for _ in range(num_init_sols)]

    m_map_bd_names = ['# modules', 'Stiffness value']
    m_map_bd_labels = (list(range(min_num_modules, settings['robot']['max_num_modules'] + 1)),
                       settings['robot']['modules_conf']['stiff_table'])

    m_map_dims = tuple([len(dim_labels) for dim_labels in m_map_bd_labels])
    c_map_dims = settings['controller']['archive_shape']

    eval_utilities = (neat_config, settings, seed, alg.get('deep_grid', False))

    pop = map_elites_double_map.run(init_sols, tbox, m_map_dims, m_map_bd_names, m_map_bd_labels, c_map_dims,
                                    morph_map_evo_file, contr_map_evo_file, history_file, morphologies,
                                    hof, settings['hof_size'], settings['result_dir'], checkpoint_dir,
                                    settings['max_num_sims'], batch_size, alg.get('mutation_weights', [1, 1, 1]),
                                    maximize_fit=False, deep_grid=alg.get('deep_grid', False),
                                    depth=alg.get('depth', 50), log_all=alg.get('log_all', False),
                                    eval_utilities=eval_utilities, verbose=settings.get('verbose', True),
                                    cp_freq=settings.get('checkpoint_freq', 5), seed=seed,
                                    dr_update_gens=alg['dr_update_gens'], alg_data=alg_data,
                                    time_no_update=time_no_update)


def end_simulation(morph_map_evo_file, contr_map_evo_file, history_file, results_dir, controllers_dir, hall_of_fame,
                   seed, morphologies, morphologies_dir, start_time, checkpoint_used):
    morph_map_evo_file.write(']')
    morph_map_evo_file.close()

    contr_map_evo_file.write(']')
    contr_map_evo_file.close()

    history_file.close()

    # copy best controller into best directory
    best_dir = os.path.join(results_dir, 'best')
    best_genome_id = hall_of_fame[min(hall_of_fame.keys())][0][1] if len(hall_of_fame.keys()) > 0 else None
    if best_genome_id is not None:
        shutil.copy2(
            os.path.join(
                controllers_dir,
                'controller_{}_{}.txt'.format(seed, best_genome_id)),
            best_dir
        )

    with open(os.path.join(morphologies_dir, 'morphologies_{}.json'.format(seed)), 'w') as morph_file:
        morph_file.write(json.dumps(morphologies, indent=4))

    with open(os.path.join(best_dir, 'hall_of_fame_{}.json'.format(seed)), 'w') as hof_file:
        hof_file.write(json.dumps(hall_of_fame, sort_keys=True, indent=4))

    # compute elapsed time
    end_time = int(time.time() - start_time)
    elap_time = '{:02d}:{:02d}:{:02d}'.format(end_time // 3600,
                                              (end_time % 3600 // 60),
                                              end_time % 60)

    # save elapsed time
    print('Total computation time: {}'.format(elap_time))
    if elap_time not in ['00:00:0{}'.format(i) for i in range(0, 6)]:
        mode = 'a' if checkpoint_used else 'w+'
        with open(os.path.join(results_dir, 'elapsed_time_sim_{}.txt'.format(seed)), mode) as time_file:
            time_file.write(elap_time + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for simulating the evolution of morphology and controller of '
                                                 ' a modular soft robot through double map elites for goal reaching '
                                                 ' or goal reaching after squeezing task')
    parser.add_argument('evo_settings', metavar='evo_settings', type=str, nargs='?',
                        default='./experiments_dm/map_elites_dm.json', help='file name of the configuration file'
                                ' containing the parameters for the simulation.' +
                                ' By default it looks for the ./experiments_dm/map_elites_dm.json file.')
    parser.add_argument('neat_settings', metavar='neat_settings', type=str, nargs='?',
                        default='./experiments_dm/neat-settings.txt',
                        help='file name of the file containing the NEAT configuration parameters used for'
                        ' controller initialization and mutation. By default it uses '
                        './experiments_dm/neat-settings.txt file.')
    parser.add_argument('--use-checkpoint', dest='use_checkpoint', action='store_const',
                        const=True, default=False, help='Select whether to run the experiment '
                        'from previous checkpoints (if exist).')

    args = parser.parse_args()
    toolbox = base.Toolbox()

    # multiprocessing pool instantiation
    pool = multiprocessing.Pool(processes=24)

    # load evolution settings
    with open(str(args.evo_settings)) as evolution_settings:
        evo_settings = json.load(evolution_settings)

    assert evo_settings['simulation_path'].endswith('Log')

    # parse settings
    results_dir, checkpoint_dir, controllers_dir, evo_dir, morphologies_dir =\
        load_evo_settings(toolbox, pool, evo_settings)

    # save settings
    with open(os.path.join(results_dir, 'settings.json'), 'w') as conf_file:
        json.dump(evo_settings, conf_file, indent=4)

    # print header
    print_header_double_map(evo_settings)

    # seeds
    seeds = evo_settings['seeds']
    for seed in seeds:
        # set init_time variable
        start_time = time.time()

        # history filename
        history_filename = os.path.join(evo_dir, 'history_'+str(seed)+'.csv')

        # evolution info files
        morph_map_evo_filename = os.path.join(evo_dir, 'morph_map_evo_{}.json'.format(seed))
        contr_map_evo_filename = os.path.join(evo_dir, 'contr_map_evo_{}.json'.format(seed))

        # checkpoint file
        checkpoint_filename = os.path.join(checkpoint_dir, 'checkpoint_{}.pkl'.format(seed))

        # checkpoint data, all morphologies and hall of fame initialization
        alg_data = None
        all_morphologies = {'_': 0}
        hall_of_fame = {}

        # init controllers genetic operators
        c_gen_operators = init_contr_genetic_operators(toolbox, evo_settings, args.neat_settings)
        if seed == seeds[0]:
            # save config file
            c_gen_operators.neat_config.save(os.path.join(results_dir, 'neat_settings.txt'))

        # if use_checkpoint option is True and the checkpoint files exist, load the populations from them
        if args.use_checkpoint and os.path.exists(checkpoint_filename):
            checkpoint_used = True

            # load checkpoint
            alg_data, last_controller_id, last_node_id, all_morphologies, hall_of_fame = \
                load_checkpoint(checkpoint_filename)
            last_checkpoint = alg_data['num_gen']

            # restore history_file
            history_file, num_sims = restore_history_file(history_filename, last_checkpoint)

            # restore evo files
            morph_map_evo_file, _ = restore_evo_file(morph_map_evo_filename, last_checkpoint, num_sims)
            contr_map_evo_file, _ = restore_evo_file(contr_map_evo_filename, last_checkpoint, num_sims)

            # adjust controller genome operators properties
            c_gen_operators.last_id = last_controller_id
            c_gen_operators.genome_indexer = count(last_controller_id+1)
            c_gen_operators.last_node_id = last_node_id
            c_gen_operators.genome_config.node_indexer = count(last_node_id+1)

            # remove controller files in excess w.r.t. checkpoint
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

            # open history file
            history_file = open_history_file(history_filename, type=2)

            # open evo files
            morph_map_evo_file = open_evo_file(morph_map_evo_filename)
            contr_map_evo_file = open_evo_file(contr_map_evo_filename)

            # delete potential controller files related to this seed
            del_c_files = [
                os.path.join(controllers_dir, f)
                for f in os.listdir(controllers_dir)
                if re.match('controller_{}_[0-9]+\.txt'.format(seed), f)
            ]
            for file in del_c_files:
                os.remove(file)

        run_evo(toolbox, evo_settings, checkpoint_dir, morph_map_evo_file, contr_map_evo_file, history_file,
                alg_data, seed, all_morphologies, hall_of_fame, robot.MIN_NUM_MODULES, c_gen_operators.neat_config)

        # perform the closing operations
        end_simulation(morph_map_evo_file, contr_map_evo_file, history_file, results_dir, controllers_dir,
                       hall_of_fame, seed, all_morphologies, morphologies_dir, start_time, checkpoint_used)

    pool.close()
