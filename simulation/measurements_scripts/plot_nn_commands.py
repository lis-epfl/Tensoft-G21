#! /usr/bin/env python3

# PLOTS HOW CABLES LENGTHS VARIATE OVER TIME

import os
import argparse
import subprocess
import numpy as np

import matplotlib
# select matplotlib backend
matplotlib.use('pdf')
import matplotlib.pyplot as plt

def load_configuration(robot_file):
    """ Read from file robot configuration
    generated for visualization

    :param robot_file: the file where the configuration is stored
    :return: a tuple containing a list of strings that represents robot modules
             and robot fitness
    """
    with open(robot_file) as in_file:
        modules = [line.strip().split(' ') for line in in_file]

    # last value is robot fitness
    return modules[:-1], float(modules[-1][-1])


def convert_config(robot_modules):
    """ Transform robot modules configuration from visualization
        format to inline format

    :param robot_modules: a list of strings that represents robot modules
    :return: a single string representing inline robot configuration
    """
    robot_string = ''

    for mod in robot_modules:
        data = np.asarray(mod)[list(range(1, 16, 2))]
        robot_string += '{} - {} - {} - {} - {} - {} - {} - {} -- '.format(*data)

    return robot_string


def plot_over_commands(data, stiff='0.1', noise='noiseless'):
    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    fig = plt.figure(figsize=(16, 15))
    ax = fig.gca()

    colors = plt.cm.viridis(np.linspace(0, 1, data.shape[1]-1))
    labels = ['10 (1)', '9 (4)', '4 (6)', '7 (7)', '3 (10)']

    for i in range(1, data.shape[1]):
        ax.plot(data[:, 0], data[:, i], 'o',
                label=labels[i-1],
                color=colors[i-1], ms=2.5)

    ax.set_title('Cable Length - Stiff: {} - Noise: {}'.format(stiff, noise), size=20, fontweight='normal', pad=50)
    ax.set_xlabel('Motor command (deg)', labelpad=15)
    ax.set_ylabel('Cable Compression', labelpad=10)
    ax.set_xlim(-2, 182)
    ax.legend(loc='center', borderaxespad=1, ncol=data.shape[1],
              frameon=False, bbox_to_anchor=(0.5, 1.03))

    plt.savefig(os.path.join('../nn_commands',
                             'cmd_compression_stiff_{}_noise_{}.pdf'.format(stiff, noise)),
                bbox_inches='tight')
    plt.close()


def plot_over_time(data, out_dir, stiff='0.1', noise='noiseless', face=''):
    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    fig = plt.figure(figsize=(16, 8))
    ax = fig.gca()

    colors = plt.cm.viridis(np.linspace(0, 1, data.shape[1]-1))
    labels = ['10 (1)', '9 (4)', '4 (6)', '7 (7)', '3 (10)']

    for i in range(1, data.shape[1]):
        ax.plot(list(range(data.shape[0])), data[:, i],
                label=labels[i-1],
                color=colors[i-1], ms=2.5)

    ax.set_title('Cable Length - Stiff: {} - Noise: {}'.format(stiff, noise), size=20, fontweight='normal', pad=50)
    ax.set_xlabel('Sim Time [ms]', labelpad=15)
    ax.set_ylabel('Cable Length [cm]', labelpad=10)
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.set_xlim(0, data.shape[0])
    ax.set_ylim(0, 6.2)
    ax.margins(0)
    ax.grid()
    ax.legend(loc='center', borderaxespad=1, ncol=data.shape[1],
              frameon=False, bbox_to_anchor=(0.5, 1.05))

    plt.savefig(os.path.join(out_dir,
                             'cable_len_{}_noise_{}_face{}.pdf'.format(stiff, noise, face)),
                bbox_inches='tight')
    plt.close()


def cable_len_over_commands(sim_output, out_dir, stiff, noise_level):
    print('Extracting data...')
    # extract from standard output the values (commands) generate by the NN
    raw_data = sim_output.stdout.decode("utf-8").strip().split('è,')[1:-1]

    filename = os.path.join(out_dir,
                            'comp_data_s_{}_n_{}.csv'.format(stiff, noise_level))

    data = np.asarray([[float(e) for e in line.split(',')] for line in raw_data])[18000:]

    min_cmd_pos = np.argmin(data[:, 0])
    max_cmd_pos = np.argmax(data[:, 0])
    selected_data = data[min_cmd_pos:max_cmd_pos+1]

    for i in range(1, selected_data.shape[1]):
        max_val = np.max(selected_data[:, i])
        selected_data[:, i] = selected_data[:, i] / max_val

    with open(filename, 'w') as out_file:
        out_file.write('cycle,N1,N4,N6,N7,N10\n')
        for l in selected_data.tolist():
            out_file.write(','.join(map(str, l)) + '\n')

    print('Plotting results...')
    plot_over_commands(selected_data, stiff=stiff, noise=noise_level)


def cable_len_over_time(sim_output, out_dir, stiff, noise_level, face):
    print('Extracting data...')
    # extract from standard output the values (commands) generate by the NN
    raw_data = sim_output.stdout.decode("utf-8").strip().split('è,')[1:-1]

    filename = os.path.join(out_dir,
                            'data_{}_noise_{}_face{}.csv'.format(stiff,
                                                                 noise_level, face))
    with open(filename, 'w') as out_file:
        out_file.write('cycle,N1,N4,N6,N7,N10\n')
        for l in raw_data:
            out_file.write(l + '\n')

    data = np.asarray([[float(e) for e in line.split(',')] for line in raw_data])

    print('Plotting results...')
    plot_over_time(data, out_dir, stiff=stiff, noise=noise_level, face=face)


def main(simulation_path, robot_file, out_dir, noise_type, noise_level, do_comparison=False):
    modules_conf, fitness = load_configuration(robot_file)
    robot_string = convert_config(modules_conf)

    # create output folder if does not exists
    os.makedirs(out_dir, exist_ok=True)

    print('Simulating...')
    exec_string = '{} {} {} {} {}'.format(simulation_path,
                                          noise_type,
                                          noise_level,
                                          0,
                                          robot_string)
    cproc = subprocess.run(exec_string.split(' '), capture_output=True)

    if do_comparison:
        cable_len_over_commands(cproc, out_dir, modules_conf[0][-1], noise_level)
    else:
        cable_len_over_time(cproc, out_dir, modules_conf[0][-1], noise_level, modules_conf[0][5])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Compute and plot the cable lengths variation over time.')
    parser.add_argument('simulation_app', metavar='simulation_app', type=str,
                        default='../apps/robotSimulation',
                        help='the program that simulate the robot')
    parser.add_argument('robot_file', metavar='robot_file', type=str,
                        help='the file where robot configuration is stored')
    parser.add_argument('out_dir', metavar='out_dir', type=str, default='../nn_commands',
                        help='the folder where results should be stored')
    parser.add_argument('noise_type', metavar='noise_type', type=int, default=0, nargs='?',
                        help='which type of noise has been employed (0: no noise; 1: gaussian; 2: uniform)')
    parser.add_argument('noise_level', metavar='noise_level', type=float, default=0.0, nargs='?',
                        help='the amount of noise introduced in the control signal')
    parser.add_argument('--comp', dest='do_comparison', action='store_const',
                        const=True, help='select whether to perform command vs compression analysis.')
    args = parser.parse_args()

    main(args.simulation_app, args.robot_file, args.out_dir,
         args.noise_type, args.noise_level, args.do_comparison)