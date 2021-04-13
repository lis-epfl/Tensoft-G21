import argparse
import json
import os
import re

from visualization.fitness_evolution import plot
from visualization_adv_tasks.plot_hof_controllers_stats import plot_hof_controllers_stats
from visualization_adv_tasks.plot_hof_morph_stats import plot_hof_morph_stats
from visualization_adv_tasks.plot_species import plot_species
from visualization_adv_tasks.visualize_controller import visualize_controller


def analyze_res(exec_type, res_dir, seed, best_file, neat_settings, hall_of_fame_file, species_file,
                contr_evo_file, chunked_evo_file, morph_evo_file, morphologies_file):
    visualize_controller(best_file, neat_settings)
    plot(contr_evo_file, chunked_file=chunked_evo_file)

    if exec_type in ['contr-evo', 'coev']:
        plot_species(species_file, res_dir, seed)

    if exec_type in ['coev', 'double-map', 'single-map']:
        plot_hof_controllers_stats(res_dir, seed, neat_settings, hall_of_fame_file, morphologies_file)
        plot_hof_morph_stats(hall_of_fame_file, morphologies_file, res_dir, seed)
        if exec_type != 'single-map':
            plot(morph_evo_file)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for plotting goal-reaching/squeezing results. It works '
                                                 'also for results obtained with double map elites algorithm.')
    parser.add_argument('res_dir', metavar='res_dir', type=str, nargs='?',
                        default='res_dir', help='the folder where results are stored')
    parser.add_argument('exec_type', metavar='exec_type', type=str, nargs='?',
                        default='contr-evo', help='Allowed values are: contr-evo, coev, double-map and single-map')
    parser.add_argument('--chunked-evo-file', dest='chunked_evo_file', action='store_const',
                        const=True, default=False, help='is the controller evo file chunked?')
    args = parser.parse_args()

    if not os.path.exists(args.res_dir):
        print('Results directory not exists!')
        exit(0)

    if not args.exec_type in ['contr-evo', 'coev', 'double-map', 'single-map']:
        print('Invalid execution type!')
        exit(0)

    settings_file = os.path.join(args.res_dir, 'settings.json')
    with open(settings_file) as sf:
        settings = json.load(sf)
    neat_settings = os.path.join(args.res_dir, 'neat_settings.txt')

    for seed in settings['seeds']:
        best_dir = os.path.join(args.res_dir, 'best')
        best_files = {int(re.split('[_.]', f)[-2]): os.path.join(best_dir, f)
                      for f in os.listdir(best_dir)
                      if re.match('controller_{}_[0-9]+\.txt'.format(seed), f)}
        best_file = best_files[max(best_files.keys())]

        hall_of_fame_file = os.path.join(args.res_dir, 'best', 'hall_of_fame_{}.json'.format(seed))

        if args.exec_type in ['contr-evo', 'coev']:
            checkpoint_dir = os.path.join(args.res_dir, 'checkpoints')
            species_files = {int(f.split('_')[-1]): os.path.join(checkpoint_dir, f)
                             for f in os.listdir(checkpoint_dir)
                             if re.match('contr_stats_cp_{}_[0-9]+'.format(seed), f)}
            species_file = species_files[max(species_files.keys())]
        else:
            species_file = None

        if args.exec_type in ['contr-evo', 'coev', 'double-map']:
            contr_evo_file = os.path.join(
                args.res_dir,
                'evolution_info',
                'contr{}_evo_{}.json'.format('_map' if args.exec_type == 'double-map' else '', seed)
            )
            if args.chunked_evo_file:
                contr_evo_file_chunks = [
                    os.path.join(args.res_dir, 'evolution_info', f)
                    for f in os.listdir(os.path.join(args.res_dir, 'evolution_info'))
                    if re.match(
                        'contr{}_evo_{}_[0-9]+.json'.format('_map' if args.exec_type == 'double-map' else '', seed),
                        f
                    )
                ]
                contr_evo_file_chunks.sort(key=lambda x: int(re.split('_|\.', x)[-2]))
                contr_evo_file = ([contr_evo_file] if os.path.exists(contr_evo_file) else []) + contr_evo_file_chunks

            morph_evo_file = os.path.join(
                args.res_dir,
                'evolution_info',
                'morph{}_evo_{}.json'.format('_map' if args.exec_type == 'double-map' else '', seed))
        else:
            contr_evo_file = os.path.join(
                args.res_dir,
                'evolution_info',
                'map_evo_{}.json'.format(seed)
            )
            morph_evo_file = None

        morphologies_file = os.path.join(args.res_dir, 'morphologies', 'morphologies_{}.json'.format(seed))

        analyze_res(args.exec_type, args.res_dir, seed, best_file, neat_settings, hall_of_fame_file, species_file,
                    contr_evo_file, args.chunked_evo_file, morph_evo_file, morphologies_file)
