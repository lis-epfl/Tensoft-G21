import argparse
import json
import multiprocessing
import numpy as np
import os
import pandas as pd
import re
import subprocess
import sys

from deap import base, creator

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import robot

from double_map.utils_double_map import load_checkpoint
from params_conf import N_MODULES, MIN_NUM_MODULES, STIFF_TABLE
from utils import parse_robot_string

creator.create('FitnessMax', base.Fitness, weights=(1.0,))
creator.create('Robot', robot.Robot, fitness=creator.FitnessMax)


def re_exec_sim(sim_data):
    morph_id, morph_string, contr_id, contr_file, fit_string, settings, tmp_contr_data_dir = sim_data

    target_dist_bearing = settings['target_dist_bearing']
    robot_tests = settings['robot']['robot_tests']

    # load simulation data from controller file
    direction, rot_angle, sim_seeds = 0, 0, []
    with open(contr_file) as cf:
        lines = cf.readlines()
        for i in range(0, len(lines)):
            elements = lines[i].split(' ')
            if elements[0] == 'fitness:':
                m_id = int(lines[i + 1].split()[1])
                if m_id == morph_id and fit_string == '{:.4f}'.format(float(lines[i].split()[-1])):
                        direction = int(lines[i + 4].split()[1])

                        if 'SGR' in settings['simulation_path']:
                            rot_angle = lines[i + 5].split()[1]
                            first_t_line_indx = i + 7
                        else:
                            first_t_line_indx = i + 6

                        for j in range(first_t_line_indx, first_t_line_indx+len(target_dist_bearing)):
                            target_line = lines[j].split()

                            for k in range(0, robot_tests):
                                sim_seed = int(target_line[7 + k + 1].replace(',', ''))
                                sim_seeds.append(sim_seed)

    if direction == 0 or ('SGR' in settings['simulation_path'] and rot_angle == 0) or sim_seeds == []:
        raise Exception(
            'Error, simulation data not found in controller file (m_id: {}, c_file: {})'.format(morph_id, contr_file)
        )

    # execute simulations
    data_file_paths = []
    for target_index in range(len(target_dist_bearing)):
        dist_bearing = target_dist_bearing[target_index]

        for i in range(0, robot_tests):
            sim_index = target_index * robot_tests + i

            data_file_name = 'controller_data_{}_{}_{}_{}_{}.csv'.format(morph_id, contr_id,
                dist_bearing[0], dist_bearing[1], sim_seeds[sim_index])
            data_file_path = os.path.join(tmp_contr_data_dir, data_file_name)
            data_file_paths.append(data_file_path)

            try:
                exec_string = '{} {} {} {} {} {} {}'.format(settings['simulation_path'], settings['sim_time'],
                    settings['noise_type'], settings['noise_level'], sim_seeds[sim_index], contr_file, direction)

                if 'SGR' in settings['simulation_path'] and rot_angle is not None:
                    exec_string = '{} {}'.format(exec_string, rot_angle)

                exec_string = '{} {} {} {} {} {}'.format(exec_string, dist_bearing[0], dist_bearing[1],
                                                   data_file_path, 0, morph_string)

                c_proc = subprocess.run(exec_string.split(' '), capture_output=True)
                # exec_out = re.split(' |\n', c_proc.stdout.decode("utf-8").strip())
            except:
                raise Exception('An error occurred during the simulation!\n'
                                + 'Please run this {} and check the result'.format(exec_string))

    # extract sensory data
    sensory_data = []
    for i in range(0, len(target_dist_bearing)):
        target_trajectories = []

        for j in range(0, robot_tests):
            data_file_path = data_file_paths[i * robot_tests + j]

            df = pd.read_csv(data_file_path)
            df = df.filter(items=['transf_x', 'transf_z'])

            num_rows = int(len(df) / 1000)
            row_indices = [k * int(len(df) / num_rows) for k in range(1, num_rows + 1)]
            data = df.iloc[row_indices, :]
            data = np.concatenate(([[0, 0]], data.values), axis=0)

            trajectory = np.array([data[k + 1] - data[k] for k in range(0, len(data) - 1)])
            target_trajectories.append(trajectory)

            os.remove(data_file_path)

        target_avg_trajectory = sum(target_trajectories) / len(target_trajectories)
        target_avg_trajectory = [item for pair in target_avg_trajectory for item in pair]

        sensory_data += target_avg_trajectory

    return sensory_data


def load_contr_archive(archive_file, bounds_file, deep_grid, all_deep_indiv, best_function, inv_m_dict):
    with open(bounds_file) as boundaries_file:
        bounds = json.load(boundaries_file)

        c_archive = np.full(shape=(len(bounds[0])-1, len(bounds[1])-1), fill_value=None)
        with open(archive_file) as af:
            _ = af.readline()  # skip header
            for line in af:
                index_1, index_2, value_1, value_2, fit, m_id, c_id = line.split(',')
                index_1, index_2, fit, m_id, c_id = int(index_1), int(index_2), float(fit), int(m_id), int(c_id)

                if deep_grid and all_deep_indiv:
                    if c_archive[index_1, index_2] is None:
                        c_archive[index_1, index_2] = [(index_1, index_2, value_1, value_2, fit, m_id, c_id)]
                    else:
                        c_archive[index_1, index_2] += [(index_1, index_2, value_1, value_2, fit, m_id, c_id)]
                elif not deep_grid or c_archive[index_1, index_2] is None \
                                   or fit == best_function(fit, c_archive[index_1, index_2][0][4]):
                    c_archive[index_1, index_2] = [(index_1, index_2, value_1, value_2, fit, m_id, c_id)]

        single_archive = np.full(
            shape=(len(N_MODULES), len(STIFF_TABLE), len(bounds[0]) - 1, len(bounds[1]) - 1),
            fill_value=None
        )

        for i in range(c_archive.shape[0]):
            for j in range(c_archive.shape[1]):
                if c_archive[i, j] is not None:
                    for individual in c_archive[i, j]:
                        index_3, index_4, value_3, value_4, fit, m_id, c_id = individual

                        robot = parse_robot_string(inv_m_dict[m_id])
                        num_modules = len(robot)
                        stiffness = robot[0]['stiff']

                        index_1 = num_modules - MIN_NUM_MODULES
                        index_2 = STIFF_TABLE.index(stiffness)

                        if single_archive[index_1, index_2, index_3, index_4] is None \
                                or fit == best_function(fit, single_archive[index_1, index_2, index_3, index_4][8]):
                            single_archive[index_1, index_2, index_3, index_4] = (
                                index_1, index_2, index_3, index_4,
                                num_modules, stiffness, value_3, value_4,
                                fit, m_id, c_id
                            )

        return single_archive, bounds


def merge_morph_archive(archive, m_archive_file, checkpoint_file, contr_dir, tmp_contr_data_dir,
                        inv_m_dict, settings, seed, best_function):
    alg_data, _, _, _, _ = load_checkpoint(checkpoint_file)
    dr_model = alg_data['dr_model']

    m_archive_l = []
    with open(m_archive_file) as af:
        _ = af.readline()  # skip header
        for line in af:
            index_1, index_2, val_1, val_2, fit, m_id, c_id = line.split(',')
            fit_string = fit
            index_1, index_2, fit, m_id, c_id = int(index_1), int(index_2), float(fit), int(m_id), int(c_id)

            m_archive_l.append((index_1, index_2, val_1, val_2, fit, m_id, c_id, fit_string))

    pool = multiprocessing.Pool(processes=3)

    pool_input = []
    for index_1, index_2, val_1, val_2, fit, m_id, c_id, fit_string in m_archive_l:
        pool_input.append((
            m_id,
            inv_m_dict[m_id],
            c_id,
            os.path.join(contr_dir, 'controller_{}_{}.txt'.format(seed, c_id)),
            fit_string,
            settings,
            tmp_contr_data_dir
        ))

    sensory_data = pool.map(re_exec_sim, pool_input)
    pool.close()

    sd_df = pd.DataFrame(sensory_data)
    c_bds, bounds_update = dr_model.predict(sd_df)
    assert bounds_update is False

    for individual, sensory_data, c_bd in zip(m_archive_l, sensory_data, c_bds):
        index_1, index_2, value_1, value_2, fit, m_id, c_id, _ = individual

        index_3, index_4 = c_bd
        value_3, value_4 = dr_model.predicted_vals([sensory_data])[0]

        if archive[index_1, index_2, index_3, index_4] is None \
                or fit == best_function(fit, archive[index_1, index_2, index_3, index_4][8]):
            archive[index_1, index_2, index_3, index_4] = (
                index_1, index_2, index_3, index_4,
                value_1, value_2, value_3, value_4,
                fit, m_id, c_id
            )

    return archive


def merge_archives(m_archive_file, c_archive_file, c_bounds_archive_file, morphologies_file, deep_grid, all_deep_indiv,
                   best_function, checkpoint_file, contr_dir, tmp_contr_data_dir, seed, settings,
                   archives_dir, num_gen):
    with open(morphologies_file) as morph_file:
        morphologies = json.load(morph_file)
    inverse_morph_dict = {v: k for k, v in morphologies.items()}

    archive, bounds = load_contr_archive(c_archive_file, c_bounds_archive_file,
                                         deep_grid, all_deep_indiv, best_function, inverse_morph_dict)

    archive = merge_morph_archive(archive, m_archive_file, checkpoint_file, contr_dir, tmp_contr_data_dir,
                                  inverse_morph_dict, settings, seed, best_function)

    with open(os.path.join(archives_dir, 'archive_{}_ngen_{}.csv'.format(seed, num_gen)), 'w') as out_file:
        out_file.write('1st_dim_indx,2nd_dim_indx,3rd_dim_indx,4th_dim_indx,'
                       '1st_dim:num_modules,2nd_dim:stiffness_value,3rd_dim_val,4th_dim_val,fitness,e_id,nn_id\n')

        for i in range(0, archive.shape[0]):
            for j in range(0, archive.shape[1]):
                for k in range(0, archive.shape[2]):
                    for l in range(0, archive.shape[3]):
                        entry = archive[i, j, k, l]
                        if entry is not None:
                            index_1, index_2, index_3, index_4, value_1, value_2, value_3, value_4, fit, m_id, c_id = \
                                entry
                            out_file.write('{},{},{},{},{},{},{:.4f},{:.4f},{:.4f},{},{}\n'.format(
                                index_1, index_2, index_3, index_4, value_1, value_2, float(value_3), float(value_4),
                                fit, m_id, c_id
                            ))

    with open(os.path.join(archives_dir, 'nn_section_bounds_{}_ngen_{}.json'.format(seed, num_gen)), 'w+') as file:
        json.dump(bounds, file)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for merging a double archive into a single one')
    parser.add_argument('res_dir', metavar='res_dir', type=str, nargs='?',
                        help='folder containing evolution results')
    parser.add_argument('--deep-grid', metavar='deep-grid', type=bool, action='store',
                        default=False, help='is this a deep grid execution?')
    parser.add_argument('--all-deep-indiv', metavar='all-deep-indiv', type=bool, action='store',
                        default=False, help='should be considered all individuals in each deep cell?')
    parser.add_argument('--fit-func-best', metavar='fitness_function_best', type=str, nargs='?', default='min',
                        help='function used to determine the best fitness, allowed values are: min, max')
    args = parser.parse_args()

    if args.fit_func_best not in ['min', 'max']:
        raise Exception('The function provided to determine the best fitness'
                        + 'is not valid')
    else:
        fit_func_best = min if args.fit_func_best == 'min' else max

    if args.res_dir is not None:
        settings_file = os.path.join(args.res_dir, 'settings.json')
        with open(settings_file) as sf:
            settings = json.load(sf)

        abs_res_dir_path = os.path.abspath(args.res_dir)
        archives_dir = os.path.join(abs_res_dir_path, 'archives')
        controllers_dir = os.path.join(abs_res_dir_path, 'controllers')
        tmp_contr_data_dir = os.path.join(abs_res_dir_path, 'tmp_contr_data')
        for seed in settings['seeds']:
            morph_archive_files = {int(re.split('_|\.', f)[-2]): os.path.join(archives_dir, f)
                                   for f in os.listdir(archives_dir)
                                   if re.match('entity_archive_{}_ngen_[0-9]+\.csv'.format(seed), f)}
            num_gen = max(morph_archive_files.keys())
            morph_archive_file = morph_archive_files[num_gen]

            contr_archive_files = {int(re.split('_|\.', f)[-2]): os.path.join(archives_dir, f)
                                   for f in os.listdir(archives_dir)
                                   if re.match('nn_archive_{}_ngen_[0-9]+\.csv'.format(seed), f)}
            contr_archive_file = contr_archive_files[max(contr_archive_files.keys())]

            contr_archive_bounds_files = {int(re.split('_|\.', f)[-2]): os.path.join(archives_dir, f)
                                          for f in os.listdir(archives_dir)
                                          if re.match('nn_archive_bounds_{}_ngen_[0-9]+\.json'.format(seed), f)}
            contr_archive_bounds_file = contr_archive_bounds_files[max(contr_archive_bounds_files.keys())]

            morphologies_file = os.path.join(abs_res_dir_path, 'morphologies', 'morphologies_{}.json'.format(seed))

            checkpoint_file = os.path.join(abs_res_dir_path, 'checkpoints', 'checkpoint_{}.pkl'.format(seed))

            merge_archives(morph_archive_file, contr_archive_file, contr_archive_bounds_file,
                           morphologies_file, args.deep_grid, args.all_deep_indiv, fit_func_best,
                           checkpoint_file, controllers_dir, tmp_contr_data_dir, seed, settings,
                           archives_dir, num_gen)