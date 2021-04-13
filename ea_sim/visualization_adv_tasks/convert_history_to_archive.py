import argparse
import json
import numpy as np
import os
import subprocess
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from params_conf import N_MODULES, MIN_NUM_MODULES, STIFF_TABLE
from utils import parse_robot_string


def convert_h_to_arch(h_file, morph_file, best_function, out_dir, seed):
    with open(morph_file) as morph_file:
        morphologies = json.load(morph_file)
    inverse_morph_dict = {v: k for k, v in morphologies.items()}

    archive = np.full(shape=(len(N_MODULES), len(STIFF_TABLE)), fill_value=None)
    last_gen = 0

    with open(h_file) as hf:
        _ = hf.readline()  # skip header
        for line in hf:
            last_gen, m_id, c_id, fit, _ = line.split(',')
            last_gen, m_id, c_id, fit = int(last_gen), int(m_id), int(c_id), float(fit)

            robot = parse_robot_string(inverse_morph_dict[m_id])

            index_1 = len(robot) - MIN_NUM_MODULES
            index_2 = STIFF_TABLE.index(robot[0]['stiff'])

            value_1 = len(robot)
            value_2 = robot[0]['stiff']

            if archive[index_1, index_2] is None or fit == best_function(archive[index_1, index_2][4], fit):
                archive[index_1, index_2] = (index_1, index_2, value_1, value_2, fit, m_id, c_id)

    out_filename = os.path.join(out_dir, 'entity_archive_{}_ngen_{}.csv'.format(seed, last_gen))
    with open(out_filename, 'w') as out_file:
        out_file.write('1st_dim_indx,2nd_dim_indx,1st_dim:num_modules,2nd_dim:stiffness_value,fitness,e_id,nn_id\n')
        for entry in archive.flatten():
            if entry is not None:
                index_1, index_2, value_1, value_2, fit, m_id, c_id = entry
                out_file.write('{},{},{},{},{:.4f},{},{}\n'.format(
                    index_1, index_2, value_1, value_2, fit, m_id, c_id
                ))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for converting an history file into a archive based on '
                                                 'morphological features.')
    parser.add_argument('res_dirs', metavar='res_dirs', type=str, nargs='+',
                        help='list of folders containing evolution results')
    parser.add_argument('--fit-func-best', metavar='fitness_function_best', type=str, nargs='?', default='min',
                        help='function used to determine the best fitness, allowed values are: min, max')
    parser.add_argument('--owner', metavar='owner', type=str, action='store', nargs='+',
                        default=None, help='User and group for chown')
    args = parser.parse_args()

    if args.fit_func_best not in ['min', 'max']:
        raise Exception('The function provided to determine the best fitness'
                        + 'is not valid')
    else:
        fit_func_best = min if args.fit_func_best == 'min' else max

    if args.res_dirs is not None:
        for res_dir in args.res_dirs:
            settings_file = os.path.join(res_dir, 'settings.json')
            with open(settings_file) as sf:
                settings = json.load(sf)

            abs_res_dir_path = os.path.abspath(res_dir)
            evo_info_dir = os.path.join(abs_res_dir_path, 'evolution_info')

            out_dir = os.path.join(abs_res_dir_path, 'archives')
            os.makedirs(out_dir, exist_ok=True)

            for seed in settings['seeds']:
                history_file = os.path.join(evo_info_dir, 'history_{}.csv'.format(seed))
                morphologies_file = os.path.join(abs_res_dir_path, 'morphologies', 'morphologies_{}.json'.format(seed))

                convert_h_to_arch(history_file, morphologies_file, fit_func_best, out_dir, seed)

            if args.owner is not None and len(args.owner) == 2:
                try:
                    exec_string = 'chown -R {}:{} {}'.format(args.owner[0], args.owner[1], out_dir)
                    c_proc = subprocess.run(exec_string.split(' '), capture_output=True)
                except:
                    raise Exception('An error occurred during the owner setting')