import argparse
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import os


def plot_controller_data(data_file):
    data = pd.read_csv(data_file, skiprows=[1])

    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300
    colors = plt.cm.viridis(np.linspace(0, 1, 1))

    make_plot(data, 1, 'Distance', colors[0], data_file)
    make_plot(data, 2, 'Bearing', colors[0], data_file)


def make_plot(data, col_index, label, color, filename):
    fig = plt.figure(figsize=(12, 8))
    ax = fig.gca()

    sim_time = data.iloc[:, 0].tolist()
    data_to_plot = data.iloc[:, col_index].tolist()

    ax.plot(sim_time, data_to_plot, label=label, color=color)

    ax.set_title('Controller ', size=20, fontweight='normal')
    ax.set_xlabel('Simulation Time', labelpad=15)
    ax.set_ylabel(label+' Value', labelpad=10)
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.set_xlim(3, sim_time[-1])
    ax.set_ylim(min(data_to_plot)-1, max(data_to_plot) + 1)
    ax.margins(0)
    ax.grid()
    ax.legend(loc=8, borderaxespad=1, ncol=3, frameon=False)

    out_file = filename[0:filename.rfind('.')] + '_' + label.lower() + '.pdf'
    os.makedirs(os.path.dirname(os.path.normpath(out_file)), exist_ok=True)
    plt.savefig(out_file, bbox_inches='tight')
    plt.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for plotting controller inputs (distance and bearing) '
                                                 'vs time.')
    parser.add_argument('controller_data_file', metavar='controller_data_file', type=str, nargs='?',
                        default='controller_data.csv',
                        help='the controller data file')
    args = parser.parse_args()

    plot_controller_data(args.controller_data_file)
