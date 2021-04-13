import argparse
import json
import os
import subprocess

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for executing a specific coev simulation (visual or not). '
                                                 'It works also for simulations related to double/single map '
                                                 'evolution.')
    parser.add_argument('results_dir', metavar='results_dir', type=str, nargs='?',
                        default='results_dir', help='the results directory')
    parser.add_argument('morphology_id', metavar='morphology_id', type=int, nargs='?',
                        default=1, help='id of the morphology to simulate')
    parser.add_argument('controller_id', metavar='controller_id', type=int, nargs='?',
                        default=1, help='id of the controller to simulate')
    parser.add_argument('seed', metavar='seed', type=int, nargs='?',
                        default=0, help='seed of the simulation')
    parser.add_argument('target_number', metavar='target_number', type=int, nargs='?',
                        default=1, help='number of target-bearing pair (from 1 to len(targets))')
    parser.add_argument('sim_seed_number', metavar='sim_seed_number', type=int, nargs='?',
                        default=1, help='number of simulator seed (from 1 to len(sim_seeds))')
    parser.add_argument('mode', metavar='mode', type=str, nargs='?',
                        default='s', help='execution mode: simulation (s) or visualization (v)')
    parser.add_argument('--sim-time', dest='sim_time', metavar='sim_time',
                        type=int, nargs='?', help='Simulation time')
    parser.add_argument('--log-file', dest='log_file', metavar='log_file',
                        type=str, nargs='?', help='Log file path')
    args = parser.parse_args()

    res_dir = args.results_dir
    settings_file = os.path.join(res_dir, 'settings.json')
    morph_file = os.path.join(os.path.join(res_dir, 'morphologies'), 'morphologies_{}.json'.format(args.seed))
    contr_file = os.path.join(os.path.join(res_dir, 'controllers'),
                              'controller_{}_{}.txt'.format(args.seed, args.controller_id))

    if os.path.exists(res_dir) and os.path.exists(settings_file) and \
            os.path.exists(morph_file) and os.path.exists(contr_file):
        with open(settings_file) as sf:
            settings = json.load(sf)

        with open(morph_file) as mf:
            morphologies = json.load(mf)

        num_modules, rb_init_pos, direction, rot_angle, target_pos, sim_seed = 0, (), 0, None, (), None
        with open(contr_file) as cf:
            lines = cf.readlines()

            for i in range(0, len(lines)):
                elements = lines[i].split(' ')

                if elements[0] == 'fitness:':
                    m_id = int(lines[i+1].split()[1])

                    if m_id == args.morphology_id:
                        num_modules = int(lines[i+2].split()[1])

                        init_pos_line = lines[i+3].split()
                        rb_init_pos = (init_pos_line[1], init_pos_line[2])

                        direction = int(lines[i+4].split()[1])

                        if 'SGR' in settings['simulation_path']:
                            rot_angle = lines[i+5].split()[1]
                            target_line = lines[i+6+args.target_number].split()
                        else:
                            target_line = lines[i+5+args.target_number].split()

                        target_pos = (target_line[5], target_line[6])

                        if target_line[7] == 'sim_seeds:':
                            sim_seed = int(target_line[7 + args.sim_seed_number].replace(',', ''))

        if num_modules != 0 and rb_init_pos != () and direction != 0 and target_pos != ():

            if args.sim_time is None:
                if 'sim_time' in settings:
                    sim_time = settings['sim_time']
                else:
                    sim_time = 20
            else:
                sim_time = args.sim_time

            # support for previous versions
            if sim_seed is None:
                sim_seed = settings['robot']['sim_seeds'][args.sim_seed_number - 1] \
                    if 'sim_seeds' in settings['robot'] \
                    else settings['robot']['sim_seed']

            target_d_b = settings['target_dist_bearing'][args.target_number - 1]

            inverse_morph_dict = {v: k for k, v in morphologies.items()}

            if args.mode == 's':
                try:
                    sim_path = settings['simulation_path']
                    if args.log_file:
                        if not sim_path.endswith('Log'):
                            sim_path += 'Log'
                    else:
                        if sim_path.endswith('Log'):
                            sim_path = sim_path[:-3]

                    exec_string = '{} {} {} {} {} {} {}'.format(sim_path,
                                                                sim_time,
                                                                settings['noise_type'],
                                                                settings['noise_level'],
                                                                sim_seed,
                                                                contr_file,
                                                                direction)

                    if 'SGR' in settings['simulation_path'] and rot_angle is not None:
                        exec_string = '{} {}'.format(exec_string, rot_angle)

                    exec_string = '{} {} {}'.format(exec_string,
                                                    target_d_b[0],
                                                    target_d_b[1])

                    if args.log_file:
                        exec_string = '{} {}'.format(exec_string,
                                                     args.log_file)

                    exec_string = '{} {} {}'.format(exec_string,
                                                    0,  # robot initial position
                                                    inverse_morph_dict[args.morphology_id])

                    c_proc = subprocess.run(exec_string.split(' '))
                except:
                    raise Exception('An error occurred during the simulation!\n'
                                    + 'Please run this {} and check the result'.format(exec_string))
            elif args.mode == 'v':
                try:
                    exec_string = '{} in {} {} {} {} {}'.format(
                        settings['visualization_path'],
                        settings['noise_type'],
                        settings['noise_level'],
                        sim_seed,
                        contr_file,
                        direction)

                    if 'SGR' in settings['simulation_path'] and rot_angle is not None:
                        exec_string = '{} {}'.format(exec_string, rot_angle)

                    exec_string = '{} {} {}'.format(exec_string,
                                                    target_pos[0],
                                                    target_pos[1])

                    if 'SGR' in settings['visualization_path']:
                        exec_string = '{} {} {} {} {}'.format(exec_string,
                                                              rb_init_pos[0],
                                                              rb_init_pos[1],
                                                              num_modules,
                                                              target_d_b[1])

                    exec_string = '{} {} {}'.format(exec_string,
                                                    0,  # robot initial position
                                                    inverse_morph_dict[args.morphology_id])

                    c_proc = subprocess.run(' '.join(exec_string.split()).split(' '))
                except:
                    raise Exception('An error occurred during the simulation!\n'
                                    + 'Please run this {} and check the result'.format(exec_string))
            else:
                print('Error! Undefined mode {}!'.format(args.mode))

        else:
            print('Error! Malformed controller file!')
            print('Num modules: {}, robot init pos: {}, direction: {}, target pos: {}'.format(
                num_modules, rb_init_pos, direction, target_pos
            ))
    else:
        print('Error! Directories not exist!')
