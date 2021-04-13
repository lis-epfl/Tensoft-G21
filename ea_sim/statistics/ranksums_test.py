#! /usr/bin/env python3

import os
import re
import argparse
from itertools import combinations
from scipy.stats import ranksums

def exp_name(p):
    return os.path.basename(os.path.dirname(os.path.normpath(p)))


def load_fitness_from_best(file):
    with open(file, 'r') as best_file:
        fitness = float([line for line in best_file][-1].strip().split(':')[-1])
    return fitness


def main(best_folds, out_file):

    num_runs = 10 # maximum number of run executed per experiment
    alg_fits = {}
    for fold in best_folds:
        best_files = [f for f in os.listdir(fold) if re.match('^robot.*\.txt', f)]
        tmp_data = [(int(bf.split('_')[-1][0]),
                                     load_fitness_from_best(os.path.join(fold, bf)))
                                    for bf in best_files]
        results = {k: 0.0 for k in range(num_runs)}
        for fit in tmp_data:
            results[fit[0]] = max(results[fit[0]], fit[1])

        alg_fits[exp_name(fold)] = list(results.values())[:5]   # keep only first 5, because only 5 exp executed

    for alg, values in alg_fits.items():
        print(alg, values, '\n')

    combs = list(combinations(alg_fits.keys(), 2))
    with open(out_file, 'w') as of:
        for c in combs:
            print('\n', c)
            rs = ranksums(alg_fits[c[0]], alg_fits[c[1]])
            print(rs)
            of.write(str(rs) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for plotting the progress and results'
                                                 'of the evolutionary algorithm over different experiments')
    parser.add_argument('best_folders', metavar='best_folders', type=str, nargs='+',
                        help='list of folders where best robot designs are held')
    parser.add_argument('--out-file', metavar='out-file', type=str, action='store',
                        default='ranksum.txt', help='output filename where test results are stored.')

    args = parser.parse_args()
    main(args.best_folders, args.out_file)
