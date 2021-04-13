import os
import re
import argparse
from numpy import asarray, mean, float64, linspace, arange, sign

import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt

START_TIME = 160000

def main(measures_file, out_dir=None):
    with open(measures_file) as in_file:
        data = asarray([line.strip().split(',')
                        for line in in_file][START_TIME:], dtype=float64)[:, 1:]

    averages = mean(data, axis=0)
    print('Cables len:', averages)

    if out_dir is None:
        out_dir = os.path.dirname(measures_file)
    f_name = '_'.join(['cables_est'] + os.path.basename(measures_file).split('_')[1:])
    with open(os.path.join(out_dir, f_name), 'w') as out_file:
        out_file.write('N1,N4,N6,N7,N10\n')
        out_file.write(','.join(map(str, averages)) + '\n')


def autolabel(ax, rects):
    for i, rect in enumerate(rects):
        height = round(rect.get_height(), 3)
        ax.annotate('{}'.format(height),
                    xy=(rect.get_x() + rect.get_width()/2,
                        height + height/100 * sign(height)),
                    xytext=(0, 3 * sign(height)),
                    textcoords="offset points",
                    ha='center', va='bottom' if height > 0 else 'top')


def plot_barchart(conf, labels, out_dir, th=None):
    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 200

    colors = plt.cm.viridis(linspace(0, 1, len(labels)))

    fig = plt.figure(figsize=(12, 7))
    ax = fig.gca()

    x = arange(len(conf['x']))
    width = 0.3

    rects = []
    for i in range(len(labels)):
        rects.append(ax.bar(x + width * (i - 1), conf['y'][i], width,
                            color=colors[i], label=labels[i]))

    if th is not None:
        ax_line = ax.twinx()

        for j in range(len(th)):
            ax_line.hlines(th[j], j-(1.5*width), j+(1.5*width),
                           colors='orangered', linestyles='dashed')
        ax_line.set_yticklabels([])
        ax_line.set_xticklabels([])
        ax_line.set_ylim(0, 6)
        ax.set_ylim(0, 6)
    else:
        # just create an alias
        ax_line = ax

    for rct in rects:
        autolabel(ax_line, rct)

    ax.set_xticks(x)
    ax.set_xticklabels(conf['x'])
    ax.set_title(conf['title'])
    ax.set_xlabel('Cables', labelpad=15, fontweight='light')
    ax.set_ylabel(conf['y_label'], labelpad=10, fontweight='light')
    ax.margins(y=0.4)
    ax.legend(loc=9, ncol=len(labels), frameon=False, title='Stiffness')

    plt.tick_params(axis='y', which='both', left=True,
                    right=False, labelleft=True, labelright=True)
    plt.savefig(os.path.join(out_dir, conf['out_file']), bbox_inches='tight')
    plt.close()

def face_average(data_dir, out_dir=None):
    nodes = ['N1', 'N4', 'N6', 'N7', 'N10']
    data_files = [f for f in os.listdir(data_dir)
                  if re.match('cables_est.+.csv', f)]

    if out_dir is None:
        out_dir = data_dir

    thresholds = [4.92627, 4.45782, 4.41734, 4.90044, 4.93764]

    # read cables len from each face (module rotation) measurements
    # and classify them according their selected stiffness
    data_per_stiff = { '0.1': [], '0.5': [], '1': [] }
    for df in data_files:
        stiff = df.split('_')[2]
        with open(os.path.join(data_dir, df)) as in_file:
            _ = in_file.readline()
            data_per_stiff[stiff].append(in_file.readline().strip().split(','))

    # compute average cables len for each stiffness value
    with open(os.path.join(out_dir, 'final_cables_len.csv'), 'w') as out_file:
        out_file.write(','.join(nodes) + '\n')
        for k, v in data_per_stiff.items():
            data_per_stiff[k] = mean(asarray(v, float64), axis=0)
            out_file.write(k + ',' + ','.join(map(str, data_per_stiff[k])) + '\n')

    average_lengths = list(data_per_stiff.values())

    plot_barchart({
        'x': nodes,
        'y': average_lengths,
        'title': 'Tensegrity Internal Cables Length',
        'out_file': 'final_cables_lengths.pdf',
        'y_label': 'Length [cm]'
    }, list(data_per_stiff.keys()), out_dir, thresholds)

    # compute the average cable len among all the stiffness values
    with open(os.path.join(out_dir, 'avg_final_cables.csv'), 'w') as out_file:
        out_file.write(','.join(nodes) + '\n')
        AL = mean(average_lengths, axis=0)
        AL_str = ','.join(map(str, AL))
        out_file.write(AL_str + '\n')
        print('AL:', AL_str)

        print('\nUse the following lines in the robotController.h file:\n')
        print ('double AL[5] = {\n\t' + str(AL[0]) + ',\n\t' + str(AL[1]) + ',\n\t'
                                      + str(AL[2]) + ',\n\t' + str(AL[3]) + ',\n\t'
                                      + str(AL[4]) + '\n};' + '\n')

    diff_data = [[], [], []]
    rel_diff_data = [[], [], []]

    # compute both the absolute difference and relative one of the cables length
    # with respect to the ones measured with stiffness 0.1
    with open(os.path.join(out_dir, 'final_cables_differences.csv'), 'w') as out_file1:
        with open(os.path.join(out_dir, 'final_cables_rel_diff_wrt0.1.csv'), 'w') as out_file2:
            for i in range(len(nodes)):
                a = (average_lengths[0][i] - thresholds[i])
                b = (average_lengths[1][i] - thresholds[i])
                c = (average_lengths[2][i] - thresholds[i])

                diff_data[0].append(a)
                diff_data[1].append(b)
                diff_data[2].append(c)
                out_file1.write(f'{nodes[i]},{a},{b},{c}\n')

                rel_diff_data[0].append(a/thresholds[i])
                rel_diff_data[1].append(b/thresholds[i])
                rel_diff_data[2].append(c/thresholds[i])
                out_file2.write(f'{nodes[i]},{rel_diff_data[0][i]},{rel_diff_data[1][i]},'
                                f'{rel_diff_data[2][i]}\n')

    plot_barchart({
        'x': nodes,
        'y': diff_data,
        'title': 'Tensegrity Internal Cables - Diff from Expected',
        'out_file': 'final_cables_diff_lengths.pdf',
        'y_label': 'Length [cm]'
    }, list(data_per_stiff.keys()), out_dir)

    plot_barchart({
        'x': nodes,
        'y': rel_diff_data,
        'title': 'Tensegrity Internal Cables - Relative Diff from Expected',
        'out_file': 'final_cables_rel_diff_lengths.pdf',
        'y_label': 'Length %'
    }, list(data_per_stiff.keys()), out_dir)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script that reads cable len from files and produce their'
                                                 'average removing the unstable period.')
    parser.add_argument('measures_file', metavar='measures_file', type=str,
                        help='either the file where measures of a single test are stored or the directory'
                        'that contains the file generated by this script (first option)'
                        ' - compute their average along the stiffness value')
    parser.add_argument('out_dir', metavar='out_dir', type=str, default=None,
                        help='the folder where results should be stored')
    args = parser.parse_args()

    # create output folder if does not exists
    if args.out_dir is not None:
        os.makedirs(args.out_dir, exist_ok=True)

    if os.path.isfile(args.measures_file):
        main(args.measures_file, args.out_dir)
    else:
        face_average(args.measures_file, args.out_dir)