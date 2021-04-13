import argparse
import json
import multiprocessing
import os
import subprocess
import sys

from math import degrees


def str_from_bearing(bearing):
    return '{}_{}'.format(
        abs(round(degrees(bearing))),
        'sx' if bearing < 0 else 'dx'
    )


def collect_data(exec_string):
    try:
        subprocess.run(exec_string.split(' '), capture_output=True)
    except:
        raise Exception('An error occurred during the simulation!\n'
                    + 'Please run this {} and check the result'.format(exec_string))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for collecting HoF controller data (it works for both '
                                                 'co-evolution and double map evolution).')
    parser.add_argument('res_dir', metavar='res_dir', type=str, nargs='?',
                        default='res_dir', help='the folder where results are stored')

    args = parser.parse_args()

    if not os.path.exists(args.res_dir):
        print('Results directory not exists!')
        exit(0)

    script_dir = os.path.join('./', os.path.dirname(sys.argv[0]))

    with open(os.path.join(args.res_dir, 'settings.json')) as settings_file:
        settings = json.load(settings_file)

    # multiprocessing pool instantiation
    pool = multiprocessing.Pool()

    hof_dir = os.path.join(args.res_dir, 'best')
    hof_stats_dir = os.path.join(args.res_dir, 'hall_of_fame_stats')
    for seed in settings['seeds']:
        with open(os.path.join(hof_dir, 'hall_of_fame_{}.json'.format(seed))) as hall_of_fame_file:
            hof = json.load(hall_of_fame_file)

        hof_controller_data_dir = os.path.join(hof_stats_dir, 'hall_of_fame_{}_contr_data'.format(seed))
        os.makedirs(hof_stats_dir, exist_ok=True)
        os.makedirs(hof_controller_data_dir, exist_ok=True)

        pool_input = []
        for pairs_list in hof.values():
            for m_id, c_id in pairs_list:
                for i in range(0, len(settings['target_dist_bearing'])):
                    for j in range(0, len(settings['robot']['robot_tests'])):
                        log_file_name = 'controller_data_{}_{}_{}_{}_{}_{}.csv'.format(
                            m_id,
                            c_id,
                            seed,
                            settings['target_dist_bearing'][i][0],
                            str_from_bearing(settings['target_dist_bearing'][i][1]),
                            j
                        )

                        log_file_path = os.path.join(
                            hof_controller_data_dir,
                            log_file_name
                        )

                        exec_string = '{} {} {} {} {} {} {} {} {} --log-file={}'.format(
                            'python',
                            os.path.join(script_dir, 'exec_coev_sim.py'),
                            args.res_dir,
                            m_id,
                            c_id,
                            seed,
                            i+1,
                            j+1,
                            's',
                            log_file_path
                        )

                        pool_input.append(exec_string)

        pool.map(collect_data, pool_input)

    pool.close()
