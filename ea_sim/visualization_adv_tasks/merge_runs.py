import argparse
import json
import os
import re
import shutil
import subprocess

from collections import defaultdict
from operator import itemgetter

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for copying goal-reaching/squeezing evolution files from '
                                                 'different runs into the same folder, which can then be used by '
                                                 'data_plotter.py to produce comparison charts.')
    parser.add_argument('root_res_dir', metavar='root_res_dir', type=str, nargs='?',
                        help='the root folder of the runs folders (e.g. ../../results')
    parser.add_argument('sub_levels_num', metavar='sub_levels_num', type=int, nargs='?',
                        help='number of levels between the root and the runs folders '
                             '(e.g. 2 if root_res_dir is ../../results and the runs folders are in '
                             '../../results/coev/gr')
    parser.add_argument('--owner', metavar='owner', type=str, action='store', nargs='+',
                        default=None, help='User and group for chown')

    args = parser.parse_args()

    if not os.path.exists(args.root_res_dir):
        print('Results directory not exists!')
        exit(0)

    dirs_list = [args.root_res_dir]
    for i in range(0, args.sub_levels_num):
        next_level_dirs_list = []
        for dir in dirs_list:
            sub_entries = os.listdir(dir)
            for sub_entry in sub_entries:
                sub_entry_path = os.path.join(dir, sub_entry)
                if os.path.isdir(sub_entry_path) and not 'all_data' in sub_entry_path:
                    next_level_dirs_list.append(sub_entry_path)
        dirs_list = next_level_dirs_list

    for dir in dirs_list:
        out_dir = '{}_all_data'.format(dir if not dir.endswith('/') else dir[:-1])
        os.makedirs(out_dir, exist_ok=True)

        all_runs_dirs = os.listdir(dir)
        all_runs_dirs.sort()
        assert len(all_runs_dirs) != 0

        index = 0
        prefix = all_runs_dirs[index].rpartition('_')[0]
        exp_out_dir = os.path.join(out_dir, prefix)
        os.makedirs(exp_out_dir, exist_ok=True)

        while index < len(all_runs_dirs):
            run_dir = all_runs_dirs[index]
            run_prefix, _, run_suffix = run_dir.rpartition('_')
            run_dir = os.path.join(dir, run_dir)
            print(run_dir)

            if run_prefix != prefix:
                prefix = run_prefix
                exp_out_dir = os.path.join(out_dir, prefix)
                os.makedirs(exp_out_dir, exist_ok=True)

            elapsed_times = {
                f: os.path.join(run_dir, f)
                for f in os.listdir(run_dir)
                if re.match('elapsed_time_sim_[0-9]+\.txt', f)
            }

            evo_info_files = {
                f: os.path.join(run_dir, 'evolution_info', f)
                for f in os.listdir(os.path.join(run_dir, 'evolution_info'))
                if f.endswith('.json')
            }

            hall_of_fames = {
                f: os.path.join(run_dir, 'best', f)
                for f in os.listdir(os.path.join(run_dir, 'best'))
                if re.match('hall_of_fame_[0-9]+\.json', f)
            }

            bests = [
                (int(re.split('_|\.', f)[-3]), int(re.split('_|\.', f)[-2]), f, os.path.join(run_dir, 'best', f))
                for f in os.listdir(os.path.join(run_dir, 'best'))
                if re.match('controller_[0-9]+_[0-9]+\.txt', f)
            ]
            bests_dict = defaultdict(list)
            for k, *v in bests:
                bests_dict[k].append(v)

            species_files = {
                (int(f.split('_')[-2]), int(f.split('_')[-1]), f, os.path.join(run_dir, 'checkpoints', f))
                for f in os.listdir(os.path.join(run_dir, 'checkpoints'))
                if re.match('contr_stats_cp_[0-9]+_[0-9]+', f)
            }
            species_dict = defaultdict(list)
            for k, *v in species_files:
                species_dict[k].append(v)

            morphologies = {}
            if os.path.exists(os.path.join(run_dir, 'morphologies')):
                morphologies = {
                    re.split('_|\.', f)[-2]: [f, os.path.join(run_dir, 'morphologies', f)]
                    for f in os.listdir(os.path.join(run_dir, 'morphologies'))
                    if re.match('morphologies_[0-9]+\.json', f)
                }

            archives = []
            single_archives = []
            if os.path.exists(os.path.join(run_dir, 'archives')):
                entity_archives = {int(re.split('_|\.', f)[-2]): [f, os.path.join(run_dir, 'archives', f)]
                                   for f in os.listdir(os.path.join(run_dir, 'archives'))
                                   if re.match('entity_archive_ngen_[0-9]+\.csv', f) or
                                      re.match('entity_archive_[0-9]+_ngen_[0-9]+\.csv', f)}
                nn_archives = {int(re.split('_|\.', f)[-2]): [f, os.path.join(run_dir, 'archives', f)]
                               for f in os.listdir(os.path.join(run_dir, 'archives'))
                               if re.match('nn_archive_ngen_[0-9]+\.csv', f) or
                                  re.match('nn_archive_[0-9]+_ngen_[0-9]+\.csv', f)}
                nn_bounds_archives = {int(re.split('_|\.', f)[-2]): [f, os.path.join(run_dir, 'archives', f)]
                                      for f in os.listdir(os.path.join(run_dir, 'archives'))
                                      if re.match('nn_archive_bounds_ngen_[0-9]+\.json', f) or
                                         re.match('nn_archive_bounds_[0-9]+_ngen_[0-9]+\.json', f)}

                if len(entity_archives) != 0 and len(nn_archives) != 0 and len(nn_bounds_archives) != 0:
                    archives = [
                        entity_archives[max(entity_archives.keys())],
                        nn_archives[max(nn_archives.keys())],
                        nn_bounds_archives[max(nn_bounds_archives.keys())]
                    ]
                elif len(entity_archives) != 0:
                    archives = [entity_archives[max(entity_archives.keys())]]

                single_archives_files = {int(re.split('_|\.', f)[-2]): [f, os.path.join(run_dir, 'archives', f)]
                                         for f in os.listdir(os.path.join(run_dir, 'archives'))
                                         if re.match('archive_[0-9]+_ngen_[0-9]+\.csv', f)}
                nn_section_bounds = {int(re.split('_|\.', f)[-2]): [f, os.path.join(run_dir, 'archives', f)]
                                     for f in os.listdir(os.path.join(run_dir, 'archives'))
                                     if re.match('nn_section_bounds_[0-9]+_ngen_[0-9]+\.json', f)}

                if len(single_archives_files) != 0 and len(nn_section_bounds) != 0:
                    single_archives = [
                        single_archives_files[max(single_archives_files.keys())],
                        nn_section_bounds[max(nn_section_bounds.keys())]
                    ]

            for file, path in elapsed_times.items():
                file_out_dir = os.path.join(exp_out_dir, 'elapsed_times')
                os.makedirs(file_out_dir, exist_ok=True)

                shutil.copy(path, os.path.join(file_out_dir, file.replace('.txt', '_{}.txt'.format(run_suffix))))

            for file, path in evo_info_files.items():
                if 'morph' in file:
                    subdir = 'evolution_info_morphologies'
                elif 'contr' in file:
                    subdir = 'evolution_info_controllers'
                else:
                    subdir = 'evolution_info'
                file_out_dir = os.path.join(exp_out_dir, subdir)
                os.makedirs(file_out_dir, exist_ok=True)

                shutil.copy(path, os.path.join(file_out_dir, file.replace('.json', '_{}.json'.format(run_suffix))))

            for file, path in hall_of_fames.items():
                file_out_dir = os.path.join(exp_out_dir, 'hall_of_fames')
                os.makedirs(file_out_dir, exist_ok=True)

                shutil.copy(path, os.path.join(file_out_dir, file.replace('.json', '_{}.json'.format(run_suffix))))

                hof_out_dir = os.path.join(file_out_dir, file.replace('.json', '_{}'.format(run_suffix)))
                os.makedirs(hof_out_dir, exist_ok=True)

                seed = re.split('_|\.', file)[-2]
                morph_file, morph_path = morphologies[seed]
                shutil.copy(morph_path, os.path.join(hof_out_dir, morph_file))

                shutil.copy(os.path.join(run_dir, 'neat_settings.txt'), hof_out_dir)

                hof_controllers_dir = os.path.join(hof_out_dir, 'controllers')
                os.makedirs(hof_controllers_dir, exist_ok=True)
                best_path, best_file = None, None

                with open(path) as hall_of_fame_file:
                    hall_of_fame = json.load(hall_of_fame_file)

                    for pair_list in hall_of_fame.values():
                        for m_id, c_id in pair_list:
                            source_path = os.path.join(run_dir, 'controllers',
                                                       'controller_{}_{}.txt'.format(seed, c_id))
                            shutil.copy(source_path, hof_controllers_dir)

                            if best_path is None:
                                best_path = source_path
                                best_file = 'controller_{}_{}.txt'.format(seed, c_id)

                bests_out_dir = os.path.join(exp_out_dir, 'bests')
                os.makedirs(bests_out_dir, exist_ok=True)
                shutil.copy(best_path,
                            os.path.join(bests_out_dir, best_file.replace('.txt', '_{}.txt'.format(run_suffix))))
                shutil.copy(os.path.join(run_dir, 'neat_settings.txt'), bests_out_dir)

            if len(hall_of_fames) == 0:
                for bests_seed, seed_items in bests_dict.items():
                    c_id, file, path = max(seed_items, key=itemgetter(0))

                    file_out_dir = os.path.join(exp_out_dir, 'bests')
                    os.makedirs(file_out_dir, exist_ok=True)

                    shutil.copy(path, os.path.join(file_out_dir, file.replace('.txt', '_{}.txt'.format(run_suffix))))
                    shutil.copy(os.path.join(run_dir, 'neat_settings.txt'), file_out_dir)

            for species_seed, seed_items in species_dict.items():
                n_gen, file, path = max(seed_items, key=itemgetter(0))

                file_out_dir = os.path.join(exp_out_dir, 'species_controllers')
                os.makedirs(file_out_dir, exist_ok=True)

                shutil.copy(path, os.path.join(file_out_dir, file+'_{}'.format(run_suffix)))

            for file, path in archives:
                if 'entity' in file:
                    subdir = 'archives_morphologies'
                else:
                    subdir = 'archives_controllers'
                file_out_dir = os.path.join(exp_out_dir, subdir)
                os.makedirs(file_out_dir, exist_ok=True)

                seed = re.split('_|\.', list(elapsed_times.keys())[0])[-2]
                if seed not in file:
                    file = file.replace('_ngen', '_{}_ngen'.format(seed))

                shutil.copy(path,
                            os.path.join(file_out_dir,
                                         file.replace('.csv', '_{}.csv'.format(run_suffix))
                                            if file.endswith('.csv')
                                            else file.replace('.json', '_{}.json'.format(run_suffix))
                            )
                )

            for file, path in single_archives:
                file_out_dir = os.path.join(exp_out_dir, 'single_archives')
                os.makedirs(file_out_dir, exist_ok=True)

                seed = re.split('_|\.', list(elapsed_times.keys())[0])[-2]
                if seed not in file:
                    file = file.replace('_ngen', '_{}_ngen'.format(seed))

                shutil.copy(path,
                            os.path.join(file_out_dir,
                                         file.replace('.csv', '_{}.csv'.format(run_suffix))
                                            if file.endswith('.csv')
                                            else file.replace('.json', '_{}.json'.format(run_suffix))
                            )
                )

            index += 1

        if args.owner is not None and len(args.owner) == 2:
            try:
                exec_string = 'chown -R {}:{} {}'.format(args.owner[0], args.owner[1], out_dir)
                c_proc = subprocess.run(exec_string.split(' '), capture_output=True)
            except:
                raise Exception('An error occurred during the owner setting')
