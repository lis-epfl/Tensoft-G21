import argparse
import json
import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
import numpy as np
import pandas as pd
import os
import re
import seaborn as sns
import subprocess
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from params_conf import N_MODULES, STIFF_TABLE


def load_single_archive(archive_file, bounds_file):
    with open(bounds_file) as boundaries_file:
        bounds = json.load(boundaries_file)

        archive = np.full(
            shape=(len(N_MODULES), len(STIFF_TABLE), len(bounds[0]) - 1, len(bounds[1]) - 1),
            fill_value=None
        )
        with open(archive_file) as af:
            _ = af.readline()  # skip header
            for line in af:
                index_1, index_2, index_3, index_4, _, _, _, _, fit, _, _ = line.split(',')
                index_1, index_2, index_3, index_4 = int(index_1), int(index_2), int(index_3), int(index_4)

                archive[index_1, index_2, index_3, index_4] = float(fit)

            return archive, bounds


def load_morph_archive(archive_file):
    archive = np.full(shape=(len(N_MODULES), len(STIFF_TABLE)), fill_value=None)
    with open(archive_file) as af:
        _ = af.readline()  # skip header
        for line in af:
            index_1, index_2, _, _, fit, _, _ = line.split(',')
            archive[int(index_1), int(index_2)] = float(fit)

    return archive


def load_contr_archive(archive_file, bounds_file, deep_grid, best_function):
    with open(bounds_file) as boundaries_file:
        bounds = json.load(boundaries_file)

        archive = np.full(shape=(len(bounds[0])-1, len(bounds[1])-1), fill_value=None)
        with open(archive_file) as af:
            _ = af.readline()  # skip header
            for line in af:
                index_1, index_2, _, _, fit, _, _ = line.split(',')
                index_1, index_2 = int(index_1), int(index_2)

                if deep_grid and archive[index_1, index_2] is not None:
                    archive[index_1, index_2] = best_function(archive[index_1, index_2], float(fit))
                else:
                    archive[index_1, index_2] = float(fit)

            return archive, bounds


def project_to_double(archive, bounds, best_function):
    m_archive = np.full(shape=(len(N_MODULES), len(STIFF_TABLE)), fill_value=None)
    c_archive = np.full(shape=(len(bounds[0]) - 1, len(bounds[1]) - 1), fill_value=None)

    for i in range(0, archive.shape[0]):
        for j in range(0, archive.shape[1]):
            for k in range(0, archive.shape[2]):
                for l in range(0, archive.shape[3]):
                    if archive[i, j, k, l] is not None:
                        if m_archive[i, j] is None:
                            m_archive[i, j] = archive[i, j, k, l]
                        else:
                            m_archive[i, j] = best_function(m_archive[i, j], archive[i, j, k, l])

                        if c_archive[k, l] is None:
                            c_archive[k, l] = archive[i, j, k, l]
                        else:
                            c_archive[k, l] = best_function(c_archive[k, l], archive[i, j, k, l])

    return m_archive, c_archive


def plot_single_archive(archive, filename, fit_diff=False, init_fit=0, invert_fit=False, min_val=None, max_val=None):
    # dimensionality
    d = archive.shape

    # fitness values
    data = np.full(shape=archive.shape, fill_value=np.NaN)
    for i in range(0, archive.shape[0]):
        for j in range(0, archive.shape[1]):
            for k in range(0, archive.shape[2]):
                for l in range(0, archive.shape[3]):
                    entry = archive[i][j][k][l]
                    if entry is not None:
                        fit_val = entry
                        if fit_diff:
                            fit_val = init_fit - fit_val
                        if invert_fit:
                            fit_val = 1.0 / fit_val

                        data[i][j][k][l] = fit_val

    # format data
    _data = np.transpose(data, axes=[1, 0, 2, 3])
    data = np.transpose(_data.reshape((d[1], d[0] * d[2], d[3])), axes=[0, 2, 1]) \
        .reshape((d[1] * d[3], d[0] * d[2]))

    # create subplots
    plt.subplots(figsize=(10, 10))

    # generate heatmaps
    df_data = pd.DataFrame(data)
    ax = sns.heatmap(df_data, mask=df_data.isnull(), annot=False,  # norm=log_norm, cbar_kws={"ticks": cbar_ticks},
                     fmt=".4f", annot_kws={'size': 10}, linewidths=.5, linecolor='grey', cmap="viridis",
                     xticklabels=False, yticklabels=False, vmin=min_val, vmax=max_val)

    # set title, invert axes and set axes labels
    # ax.set_title('MAP-Elites Feature Space', size=24, y=1.04)
    ax.invert_yaxis()
    ax.set_xlabel('# modules', size=14, labelpad=10)
    ax.set_ylabel('Stiffness', size=14, labelpad=10)

    # set ticks
    x_ticks_pos = np.linspace(d[2] / 2.0, d[0] * d[2] - d[2] / 2.0, d[2])
    y_ticks_pos = np.linspace(d[3] / 2.0, d[1] * d[3] - d[3] / 2.0, d[3])

    ax.xaxis.set_major_locator(ticker.FixedLocator(x_ticks_pos))
    ax.xaxis.set_major_formatter(ticker.FixedFormatter(N_MODULES))

    ax.yaxis.set_major_locator(ticker.FixedLocator(y_ticks_pos))
    ax.yaxis.set_major_formatter(ticker.FixedFormatter(STIFF_TABLE))

    ax.tick_params(axis=u'both', which=u'both', length=0)

    # show grid lines
    thick_grid_color = 'k'
    thick_grid_width = 1.5

    ax.vlines(
        list(range(0, d[0] * d[2] + 1, d[2])),
        *ax.get_ylim(),
        colors=thick_grid_color,
        linewidths=thick_grid_width
    )
    ax.hlines(
        list(range(0, d[1] * d[3] + 1, d[3])),
        *ax.get_xlim(),
        colors=thick_grid_color,
        linewidths=thick_grid_width
    )

    ht_figure = ax.get_figure()
    ht_figure.savefig(filename, bbox_inches='tight', dpi=400)

    plt.close()


def plot_morphologies_archive(archive, filename, fit_diff=False, init_fit=0, invert_fit=False,
                              min_val=None, max_val=None):
    matplotlib.rcParams['pdf.fonttype'] = 42
    matplotlib.rcParams['ps.fonttype'] = 42
    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    parch = np.full(shape=tuple(reversed(archive.shape)), fill_value=np.NaN)
    for i in range(archive.shape[0]):
        for j in range(archive.shape[1]):
            if archive[i, j] is not None:
                fit_val = archive[i, j]
                if fit_diff:
                    fit_val = init_fit - fit_val
                if invert_fit:
                    fit_val = 1.0 / fit_val
                parch[j, i] = fit_val


    fig = plt.figure(figsize=(12, 5.5))
    ax = fig.gca()
    ax.set_title('MAP-Elites Feature Space', size=20, fontweight='normal')
    ax.set_xlabel('# modules', labelpad=10)
    ax.set_ylabel('Stiffness', labelpad=10)

    im = plt.imshow(parch, cmap='viridis', origin='lower', vmin=min_val, vmax=max_val)

    im_ratio = parch.shape[0] / parch.shape[1]
    cbar = fig.colorbar(im, ax=ax, fraction=im_ratio * 0.047, pad=0.04)
    cbar.set_label('Fitness', rotation=270, labelpad=20)

    plt.xticks(range(len(N_MODULES)), N_MODULES)
    plt.yticks(range(len(STIFF_TABLE)), STIFF_TABLE)

    plt.savefig(filename, bbox_inches='tight')


def plot_controllers_archive(archive, boundaries, filename, fit_diff=False, init_fit=0, invert_fit=False,
                             min_val=None, max_val=None):
    matplotlib.rcParams['pdf.fonttype'] = 42
    matplotlib.rcParams['ps.fonttype'] = 42
    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300

    parch = np.full(shape=tuple(reversed(archive.shape)), fill_value=np.NaN)
    for i in range(archive.shape[0]):
        for j in range(archive.shape[1]):
            if archive[i, j] is not None:
                fit_val = archive[i, j]
                if fit_diff:
                    fit_val = init_fit - fit_val
                if invert_fit:
                    fit_val = 1.0 / fit_val
                parch[j, i] = fit_val

    fig = plt.figure(figsize=(12, 5.5))
    ax = fig.gca()
    ax.set_title('MAP-Elites Feature Space', size=20, fontweight='normal')
    ax.set_xlabel('1st dimension', labelpad=10)
    ax.set_ylabel('2nd dimension', labelpad=10)

    im = plt.imshow(parch, cmap='viridis', origin='lower', vmin=min_val, vmax=max_val)

    im_ratio = parch.shape[0] / parch.shape[1]
    cbar = fig.colorbar(im, ax=ax, fraction=im_ratio * 0.047, pad=0.04)
    cbar.set_label('Fitness', rotation=270, labelpad=20)

    if boundaries is not None:
        plt.xticks(np.linspace(-0.5, archive.shape[0]-0.5, archive.shape[0]+1),
                   ["{:.2f}".format(bound_val) for bound_val in boundaries[0]])
        plt.yticks(np.linspace(-0.5, archive.shape[1]-0.5, archive.shape[1]+1),
                   ["{:.2f}".format(bound_val) for bound_val in boundaries[1]])
    plt.savefig(filename, bbox_inches='tight')


def plot_double_heatmaps(source, source_type, out_dir, deep_grid, best_function, fit_diff, init_fit, invert_fit,
                         run_num, min_val, max_val, owner):
    if source_type == 'double':
        m_archive = load_morph_archive(source[0])
        c_archive, bounds = load_contr_archive(source[1], source[2], deep_grid, best_function)
    else:
        single_archive, bounds = load_single_archive(source[0], source[1])
        m_archive, c_archive = project_to_double(single_archive, bounds, best_function)

    if out_dir is None:
        out_dir = os.path.join(
            os.path.dirname(os.path.dirname(source[0])),
            'heatmaps'
        )
    os.makedirs(out_dir, exist_ok=True)

    filename_tokens = re.split('_|\.', source[0])
    if not run_num:
        seed, gen = int(filename_tokens[-4]), int(filename_tokens[-2])
        out_filenames = [
            'morph_heatmap_{}_gen_{}.pdf'.format(seed, gen),
            'contr_heatmap_{}_gen_{}.pdf'.format(seed, gen)
        ]
    else:
        seed, gen, run_number = int(filename_tokens[-5]), int(filename_tokens[-3]), int(filename_tokens[-2])
        out_filenames = [
            'morph_heatmap_{}_gen_{}_{}.pdf'.format(seed, gen, run_number),
            'contr_heatmap_{}_gen_{}_{}.pdf'.format(seed, gen, run_number)
        ]

    plot_morphologies_archive(m_archive, os.path.join(out_dir, out_filenames[0]),
                              fit_diff=fit_diff, init_fit=init_fit, invert_fit=invert_fit,
                              min_val=min_val, max_val=max_val)
    plot_controllers_archive(c_archive, bounds, os.path.join(out_dir, out_filenames[1]),
                             fit_diff=fit_diff, init_fit=init_fit, invert_fit=invert_fit,
                             min_val=min_val, max_val=max_val)

    if owner is not None and len(owner) == 2:
        try:
            exec_string = 'chown -R {}:{} {}'.format(owner[0], owner[1], out_dir)
            c_proc = subprocess.run(exec_string.split(' '), capture_output=True)
        except:
            raise Exception('An error occurred during the owner setting')


def plot_single_heatmap(source, source_type, out_dir, fit_diff, init_fit, invert_fit, run_num, min_val, max_val, owner):
    if source_type == 'double':
        print('This combination of source/output is not supported currently')
        exit(0)

    archive, bounds = load_single_archive(source[0], source[1])

    if out_dir is None:
        out_dir = os.path.join(
            os.path.dirname(os.path.dirname(source[0])),
            'heatmaps'
        )
    os.makedirs(out_dir, exist_ok=True)

    filename_tokens = re.split('_|\.', source[0])
    if not run_num:
        seed, gen = int(filename_tokens[-4]), int(filename_tokens[-2])
        out_filename = 'heatmap_{}_gen_{}.pdf'.format(seed, gen)
    else:
        seed, gen, run_number = int(filename_tokens[-5]), int(filename_tokens[-3]), int(filename_tokens[-2])
        out_filename = 'heatmap_{}_gen_{}_{}.pdf'.format(seed, gen, run_number)

    plot_single_archive(archive, os.path.join(out_dir, out_filename),
                        fit_diff=fit_diff, init_fit=init_fit, invert_fit=invert_fit,
                        min_val=min_val, max_val=max_val)

    if owner is not None and len(owner) == 2:
        try:
            exec_string = 'chown -R {}:{} {}'.format(owner[0], owner[1], out_dir)
            c_proc = subprocess.run(exec_string.split(' '), capture_output=True)
        except:
            raise Exception('An error occurred during the owner setting')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for generating heatmaps starting from archives.')
    parser.add_argument('--source-t', metavar='source_t', type=str, nargs='?',
                        default='double', help='type of source files, allowed values are: single, double')
    parser.add_argument('--source-dir', metavar='source_dir', type=str, nargs='?',
                        default=None, help='the folder containing the experiment results')
    parser.add_argument('--source-files', metavar='source_files', type=str, nargs='+',
                        default='source_files', help='list of source files, as alternative to source-dir (for single, '
                                                     'the archive and the bounds files are required; for double, the '
                                                     'two archives and the bounds are required)')
    parser.add_argument('--deep-grid', metavar='deep_grid', action='store_const',
                        const=True, default=False, help='is this a deep grid execution?')
    parser.add_argument('--fit-difference', dest='fit_difference', action='store_const',
                        const=True, default=False, help='show fitness values as (init_fit - fit_value)')
    parser.add_argument('--init-fit', metavar='init-fit', type=float, action='store',
                        default=45, help='initial fit value, used in combination with fit-difference')
    parser.add_argument('--invert-fit', dest='invert_fit', action='store_const',
                        const=True, default=False, help='invert fitness values')
    parser.add_argument('--min-val', metavar='min-val', type=float, action='store',
                        default=None, help='minimum value provided in the color bar')
    parser.add_argument('--max-val', metavar='max-val', type=float, action='store',
                        default=None, help='maximum value provided in the color bar')
    parser.add_argument('--out-t', metavar='out_t', type=str, nargs='?',
                        default='double', help='type of output heatmaps, allowed values are: single, double')
    parser.add_argument('--out-dir', metavar='out_dir', type=str, nargs='?',
                        default=None, help='output_dir')
    parser.add_argument('--run-num', dest='run_num', action='store_const',
                        const=True, default=False, help='do filenames include the run number?')
    parser.add_argument('--fit-func-best', metavar='fitness_function_best', type=str, nargs='?', default='min',
                        help='function used to determine the best fitness, allowed values are: min, max')
    parser.add_argument('--owner', metavar='owner', type=str, action='store', nargs='+',
                        default=None, help='User and group for chown')
    args = parser.parse_args()

    if args.source_t not in ['single', 'double'] or args.out_t not in ['single', 'double']:
        print('Wrong from-t/to-t parameter!')
        exit(0)

    if args.fit_func_best not in ['min', 'max']:
        raise Exception('The function provided to determine the best fitness'
                        + 'is not valid')
    else:
        fit_func_best = min if args.fit_func_best == 'min' else max

    if args.source_dir is not None:
        settings_file = os.path.join(args.source_dir, 'settings.json')
        with open(settings_file) as sf:
            settings = json.load(sf)

        archives_dir = os.path.join(args.source_dir, 'archives')
        for seed in settings['seeds']:
            if args.source_t == 'double':
                morph_archive_files = {int(re.split('_|\.', f)[-2]): os.path.join(archives_dir, f)
                                       for f in os.listdir(archives_dir)
                                       if re.match('entity_archive_{}_ngen_[0-9]+\.csv'.format(seed), f)}
                morph_archive_file = morph_archive_files[max(morph_archive_files.keys())]

                contr_archive_files = {int(re.split('_|\.', f)[-2]): os.path.join(archives_dir, f)
                                       for f in os.listdir(archives_dir)
                                       if re.match('nn_archive_{}_ngen_[0-9]+\.csv'.format(seed), f)}
                contr_archive_file = contr_archive_files[max(contr_archive_files.keys())]

                contr_archive_bounds_files = {int(re.split('_|\.', f)[-2]): os.path.join(archives_dir, f)
                                       for f in os.listdir(archives_dir)
                                       if re.match('nn_archive_bounds_{}_ngen_[0-9]+\.json'.format(seed), f)}
                contr_archive_bounds_file = contr_archive_bounds_files[max(contr_archive_bounds_files.keys())]

                input_files = [morph_archive_file, contr_archive_file, contr_archive_bounds_file]
            else:
                archive_files = {int(re.split('_|\.', f)[-2]): os.path.join(archives_dir, f)
                                for f in os.listdir(archives_dir)
                                if re.match('archive_{}_ngen_[0-9]+\.csv'.format(seed), f)}
                archive_file = archive_files[max(archive_files.keys())]

                nn_section_bounds_files = {int(re.split('_|\.', f)[-2]): os.path.join(archives_dir, f)
                                              for f in os.listdir(archives_dir)
                                              if re.match('nn_section_bounds_{}_ngen_[0-9]+\.json'.format(seed), f)}
                nn_section_bounds_file = nn_section_bounds_files[max(nn_section_bounds_files.keys())]

                input_files = [archive_file, nn_section_bounds_file]

            if args.out_t == 'double':
                plot_double_heatmaps(input_files, args.source_t, args.out_dir, args.deep_grid,
                                     fit_func_best, args.fit_difference, args.init_fit, args.invert_fit,
                                     args.run_num, args.min_val, args.max_val, args.owner)
            else:
                plot_single_heatmap(input_files, args.source_t, args.out_dir,
                                    args.fit_difference, args.init_fit, args.invert_fit,
                                    args.run_num, args.min_val, args.max_val, args.owner)

    elif args.source_files is not None:
        if args.source_t == 'double' and len(args.source_files) == 3:
            plot_double_heatmaps(args.source_files, args.source_t, args.out_dir, args.deep_grid,
                                 fit_func_best, args.fit_difference, args.init_fit, args.invert_fit,
                                 args.run_num, args.min_val, args.max_val, args.owner)
        elif len(args.source_files) == 2:
            plot_single_heatmap(args.source_files, args.source_t, args.out_dir,
                                args.fit_difference, args.init_fit, args.invert_fit,
                                args.run_num, args.min_val, args.max_val, args.owner)
        else:
            print('Wrong number of source files!')
    else:
        print('Input files missing!')
