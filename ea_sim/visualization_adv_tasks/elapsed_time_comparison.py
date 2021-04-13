import argparse
import math
import numpy as np
import os
import re
import subprocess
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from visualization.plot_actuation_stats import plot_stats_boxplot


def load_elapsed_time(i, file, time_um):
    print('\t\tRun #{}'.format(i))

    elapsed_hours, elapsed_minutes, elapsed_seconds = 0, 0, 0
    with open(file) as elapsed_time_file:
        for line in elapsed_time_file.readlines():
            hours, minutes, seconds = line.split(':')
            elapsed_hours += int(hours)
            elapsed_minutes += int(minutes)
            elapsed_seconds += int(seconds)

    total = elapsed_hours * 60 + elapsed_minutes + math.ceil(elapsed_seconds / 60) if time_um == 'minutes' \
        else elapsed_hours + math.ceil((elapsed_minutes + math.ceil(elapsed_seconds / 60))/60)

    return total


def load_experiment(i, folder, time_um):
    print('\tExperiment #{}'.format(i))

    # load single run elapsed times
    elapsed_time_files = [os.path.join(folder, f)
                          for f in os.listdir(folder) if re.match('elapsed_time.*\.txt', f)]
    elapsed_time_files.sort(key=lambda f: int(re.split('_|\.', f)[-2]))

    times = [load_elapsed_time(j, etf, time_um) for j, etf in enumerate(elapsed_time_files)]

    return times


def main(exps_folder, parent_folders_num, out_file, time_um, labels, owner):
    if time_um not in ['hours', 'minutes']:
        raise Exception('The provided unit of measure of time'
                        + 'is not valid: allowed values are hours and minutes!')

    print('Loading experiments...')
    fold_names, elapsed_times = list(zip(*[(os.path.basename(os.path.dirname(os.path.normpath(f))),
                                            load_experiment(i + 1, f, time_um))
                                           for i, f in enumerate(exps_folder)]))

    if labels is not None:
        if len(labels) != len(fold_names):
            raise Exception('The number of provided labels'
                            + 'is different from the number'
                            + f'of experiments: {len(labels)}!={len(fold_names)}')
    else:
        labels = list(fold_names)

    parent_folder = os.path.normpath(exps_folder[0])
    for i in range(0, parent_folders_num):
        parent_folder = os.path.dirname(parent_folder)
    out_folder = os.path.join(parent_folder, 'evo_plots')
    os.makedirs(out_folder, exist_ok=True)

    for el_time, label in zip(elapsed_times, labels):
        print('{}\n\ttimes: {}\n\tavg: {}\n\tstd: {}\n\tmin: {}\n\tmax: {}\n\tmed: {}\n'.format(
            label,
            el_time,
            np.mean(el_time),
            np.std(el_time),
            np.min(el_time),
            np.max(el_time),
            np.median(el_time)
        ))

    labels = [label.replace('#', '\n') for label in labels]
    print('Plotting results...')
    conf = {
        'data': list(elapsed_times),
        'title': 'Execution Time',
        'x_tick_labels': labels,
        'y_label': 'Time [{}]'.format(time_um),
        'ratio': False,
        'out_file': os.path.join(out_folder, out_file)
    }
    plot_stats_boxplot(conf)

    if owner is not None and len(owner) == 2:
        try:
            exec_string = 'chown -R {}:{} {}'.format(owner[0], owner[1], out_folder)
            c_proc = subprocess.run(exec_string.split(' '), capture_output=True)
        except:
            raise Exception('An error occurred during the owner setting')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for generating boxplots of execution time for different'
                                                 'experiments')
    parser.add_argument('exps_folders', metavar='exps_folders', type=str, nargs='+',
                        help='list of experiments folders containing elapsed time files')
    parser.add_argument('--parent-folders-num', metavar='parent-folders-num', type=int, action='store',
                        default=1, help='number of parent folders to traverse before creating the out folder')
    parser.add_argument('--out-file', metavar='out-file', type=str, action='store',
                        default='elapsed_times.pdf', help='output filename')
    parser.add_argument('--time-um', metavar='time-um', type=str, action='store',
                        default='minutes', help='unit of measure of time in chart, allowed values are: hours, minutes')
    parser.add_argument('--labels', metavar='labels', type=str, action='store', nargs='+',
                        default=None, help='boxplots labels - must match the number of experiments!')
    parser.add_argument('--owner', metavar='owner', type=str, action='store', nargs='+',
                        default=None, help='User and group for chown')

    args = parser.parse_args()
    main(args.exps_folders, args.parent_folders_num, args.out_file, args.time_um, args.labels, args.owner)
