import json
import numpy as np
import os
import pandas as pd
import pickle
import random

from goal_reaching.controller import Controller
from goal_reaching.robot_gr import RobotGR


def load_checkpoint(filename):
    # load evolution data from checkpoint
    with open(filename, 'rb') as cpf:
        cp_data = pickle.load(cpf)

        alg_data = {
            'last_num_sims': cp_data['num_sims'],
            'skips': cp_data['skips'],
            'num_gen': cp_data['num_gen'],
            'e_archive': cp_data['e_archive'],
            'nn_archive': cp_data['nn_archive'],
            'dr_model': cp_data['dr_model']
        }
        last_nn_id, last_node_id = cp_data['last_nn_ids']
        entity_id_map = cp_data['entity_id_map']
        hof = cp_data['hall_of_fame']
        random.setstate(cp_data['rnd_state'])

    return alg_data, last_nn_id, last_node_id, entity_id_map, hof


def evaluate_individual(sim_data):
    morph, morph_id, contr_genome, contr_genome_id, sim_seeds_seed, \
           (neat_config, evo_settings, seed, deep_grid) = sim_data

    controller = Controller(contr_genome_id, seed, contr_genome, neat_config)
    controller_path = controller.save(
        os.path.join(evo_settings['result_dir'], 'controllers'),
        check_file_existence=True
    )

    fitness = None
    sensory_data = None

    if controller_path is not None:
        r_sim = RobotGR(evo_settings['direction_path'],
                        evo_settings['rotation_angle_path']
                        if 'rotation_angle_path' in evo_settings else None,
                        morph.simulation_path, morph.tracker_path,
                        evo_settings['sim_time'],
                        morph.noise_type, morph.noise_level,
                        evo_settings['target_dist_bearing'], 0,
                        morph.num_faces, morph.num_modules, morph.robot_tests,
                        sim_seeds_seed, morph.get_modules_conf())

        tmp_contr_data_dir = os.path.join(
            evo_settings['result_dir'],
            'tmp_contr_data'
        )

        data_file_paths = []
        for dist, bearing in evo_settings['target_dist_bearing']:
            for i in range(0, morph.robot_tests):
                data_file_name = 'controller_data_{}_{}_{}_{}_{}_{}_{}.csv'.format(
                    morph_id,
                    contr_genome_id,
                    seed,
                    sim_seeds_seed,
                    dist,
                    bearing,
                    i
                )
                data_file_path = os.path.join(
                    tmp_contr_data_dir,
                    data_file_name
                )

                data_file_paths.append(data_file_path)

        fitness = r_sim.simulate(controller_path, invert_distance=False,
                                 m_id=morph_id, data_file_paths=data_file_paths)

        sensory_data = []
        for i in range(0, len(evo_settings['target_dist_bearing'])):
            target_trajectories = []

            for j in range(0, morph.robot_tests):
                data_file_path = data_file_paths[i*morph.robot_tests + j]

                df = pd.read_csv(data_file_path)
                df = df.filter(items=['transf_x', 'transf_z'])

                num_rows = int(len(df)/1000)
                row_indices = [k*int(len(df)/num_rows) for k in range(1, num_rows+1)]
                data = df.iloc[row_indices, :]
                data = np.concatenate(([[0, 0]], data.values), axis=0)

                trajectory = np.array([data[k+1]-data[k] for k in range(0, len(data)-1)])
                target_trajectories.append(trajectory)

                os.remove(data_file_path)

            target_avg_trajectory = sum(target_trajectories) / len(target_trajectories)
            target_avg_trajectory = [item for pair in target_avg_trajectory for item in pair]

            sensory_data += target_avg_trajectory

    return [fitness, sensory_data]


def update_hall_of_fame(hall_of_fame, max_size, e_id, nn_id, fitness, maximize_fit):
    hof_size = 0
    for l in hall_of_fame.values():
        hof_size += len(l)

    if hof_size < max_size:
        if fitness not in hall_of_fame:
            hall_of_fame[fitness] = [(e_id, nn_id)]
        elif (e_id, nn_id) not in hall_of_fame[fitness]:
            hall_of_fame[fitness] += [(e_id, nn_id)]
    elif (maximize_fit and fitness > min(hall_of_fame.keys())) or \
            (not maximize_fit and fitness < max(hall_of_fame.keys())):
        inserted = True
        if fitness not in hall_of_fame:
            hall_of_fame[fitness] = [(e_id, nn_id)]
        elif (e_id, nn_id) not in hall_of_fame[fitness]:
            hall_of_fame[fitness] += [(e_id, nn_id)]
        else:
            inserted = False

        if inserted:
            worst_fit = min(hall_of_fame.keys()) if maximize_fit else max(hall_of_fame.keys())

            del hall_of_fame[worst_fit][-1]
            if len(hall_of_fame[worst_fit]) == 0:
                del hall_of_fame[worst_fit]


def record_population(num_sims, population, entity_id_map, map_type, file, skips,
                      pbar=None, verbose=False, dg_partial_log=False, best_function=min):
    pop_stats = None

    if map_type == 0:  # entity-related archive
        individuals, fitness_values = [list(t) for t in zip(*population)]
    elif map_type == 1:  # nn-related archive
        if dg_partial_log:
            best_per_cell, all_individuals = [], []
            for cell in population:
                best_per_cell.append(best_function(cell, key=lambda x: x[1]))
                all_individuals += cell

            all_fitness_values = [x[1] for x in all_individuals]
            pop_stats = {
                'num_sims': num_sims,
                'avg_fitness': np.mean(all_fitness_values),
                'std_dev': np.std(all_fitness_values),
                'min': np.min(all_fitness_values),
                'max': np.max(all_fitness_values)
            }

            individuals, fitness_values, _ = [list(t) for t in zip(*best_per_cell)]
        else:
            individuals, fitness_values, _ = [list(t) for t in zip(*population)]
    else:
        individuals, fitness_values = [], []

    if pop_stats is None:
        pop_stats = {
            'num_sims': num_sims,
            'avg_fitness': np.mean(fitness_values),
            'std_dev': np.std(fitness_values),
            'min': np.min(fitness_values),
            'max': np.max(fitness_values)
        }

    # store the current population values
    file.write(json.dumps({
        **pop_stats,
        'population': [
            {
                **{'entity': {**entity.log_info(), **{'e_id': entity_id_map[entity.representation()]}}},
                **{'nn_id': nn.id, 'fitness': fitness}
            }
            for (entity, nn), fitness in zip(individuals, fitness_values)
        ]
    }))

    if verbose:
        if pbar is not None:
            pbar.set_postfix({
                'avg': pop_stats['avg_fitness'],
                'std': pop_stats['std_dev'],
                'min': pop_stats['min'],
                'max': pop_stats['max'],
                'skip': skips
            })
        else:
            print('num_sims: {} | Fitness -> avg: {} std: {} min: {} max: {}'.format(
                pop_stats['num_sims'], pop_stats['avg_fitness'],
                pop_stats['std_dev'], pop_stats['min'], pop_stats['max']
            ))
