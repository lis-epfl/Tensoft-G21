import os
import random
import numpy as np

from tqdm import tqdm
from utils import (record_population, evaluate_ind,
                   load_history, store_history, store_checkpoint)

import matplotlib
# select matplotlib backend
matplotlib.use('pdf')
import matplotlib.pyplot as plt


class Archive:
    """
        Multi-dimensional Archive of Phenotypical Elites
    """
    def __init__(self, shape, c_inc=1, c_dec=0.5):
        self.values = np.full(shape=shape, fill_value=None)
        self.curiosity = np.zeros(shape=shape, dtype=np.float32)
        self.c_inc = c_inc    # curiosity increment constant
        self.c_dec = c_dec    # curiosity decrement constant

    def add(self, individual, maximize=True):
        """ Insert a new individual in the archive,
            according to its features description,
            in case its fitness is better than the current
            elite for those features description.

        :param individual: the individual that has to be inserted into the archive
        :param maximize: whether the fitness function requires to be maximized.
                         Set to False in case it is of minimization.
        :return:
        """
        is_better = False
        b = individual.features_descriptor()

        # update mapping with new elite
        if self.values[b] is None:
            self.values[b] = individual
            # initialize curiosity when a new cell is filled
            self.curiosity[b] = self.c_inc
        else:
            if maximize:
                is_better = individual.get_fitness() > self.values[b].get_fitness()
            else:
                is_better = individual.get_fitness() < self.values[b].get_fitness()

            if is_better:
                self.values[b] = individual

        return is_better

    def to_list(self):
        """
            Return the values stored in the archive as a single
            list without considering the empty spaces
        :return:
        """
        return [e for e in self.values.flatten() if e is not None]

    def curiosity_probs(self):
        """
            Return the probability values related to how much an elite
            has provided new offspring that have been added to the archive.
            These probabilities can be employed as weights in the selection mechanism.
        """
        curiosity_scores_list = self.curiosity.flatten()
        return curiosity_scores_list[curiosity_scores_list > 0]

    def record_curiosity(self, file):
        with open(file, 'w') as out_file:
            for row in self.curiosity:
                out_file.write(','.join(map(str, row)) + '\n')

    def inc_curiosity(self, descriptor):
        """ Increase curiosity score for corresponding elite """
        self.curiosity[descriptor] += self.c_inc

    def dec_curiosity(self, descriptor):
        """ Decrease curiosity score for corresponding elite """
        self.curiosity[descriptor] = max(0.5, self.curiosity[descriptor] - self.c_dec)

    def write_map(self, filename):
        """ Write the archive as a csv file

        :param filename: the filename where to write the archive
        :return:
        """
        with open(filename, 'w') as out_file:
            out_file.write('num_modules,stiffness,fitness\n')
            for row in self.values:
                for robot in row:
                    if robot is not None:
                        out_file.write('{},{},{}\n'.format(
                            robot.num_modules,
                            robot.stiffness,
                            robot.fitness.values[0]
                        ))

    def write_3d_map(self, filename):
        """ Write the archive as a csv file

        :param filename: the filename where to write the archive
        :return:
        """
        with open(filename, 'w') as out_file:
            out_file.write('num_modules,min_stiffness,max_stiffness,fitness\n')
            for row in self.values:
                for col in row:
                    for robot in col:
                        if robot is not None:
                            stiffs = [mod.stiffness for mod in robot.modules]
                            out_file.write('{},{},{},{}\n'.format(
                                robot.num_modules,
                                np.min(stiffs),
                                np.max(stiffs),
                                robot.fitness.values[0]
                            ))

    def write_best(self, out_folder, seed, sim_id):
        """ Write all the elites contained in the archive,
            so that they can be easily visualized.

        :param out_folder: the folder where to store the individuals
        :param seed:       simulation seed
        :param sim_id:     simulation id
        :return:
        """
        for row in self.values:
            for robot in row:
                if robot is not None:
                    rob_d = robot.features_desc_str()
                    basename = 'robot_{}_seed_{}_sim_{}'.format(rob_d, seed, sim_id)
                    robot.write_input(os.path.join(out_folder, basename + '.txt'))

    def write_best_3d(self, out_folder, seed, sim_id):
        """ Write all the elites contained in the archive,
            so that they can be easily visualized.

        :param out_folder: the folder where to store the individuals
        :param seed:       simulation seed
        :param sim_id:     simulation id
        :return:
        """
        for row in self.values:
            for col in row:
                for robot in col:
                    if robot is not None:
                        rob_d = robot.features_desc_str()
                        basename = 'robot_{}_seed_{}_sim_{}'.format(rob_d, seed, sim_id)
                        robot.write_input(os.path.join(out_folder, basename + '.txt'))

    def track_best(self, trajectory_folder, seed, sim_id, num_best=10):
        """ Write all the elites contained in the archive,
            so that they can be easily visualized.

        :param trajectory_folder: the folder where to store the trajectories
        :param seed:       simulation seed
        :param sim_id:     simulation id
        :param num_best:   number of how many robots track
        :return:
        """
        best_robots = sorted(self.to_list(), reverse=True, key=lambda r: r.get_fitness())[:num_best]

        # generate robots trajectories
        for k, robot in enumerate(best_robots):
            print(f'\rRobot: {k+1}/{len(best_robots)}', end='')
            rob_d = robot.features_desc_str()
            filename = 'robot_{}_{}_seed_{}_sim_{}'.format(k, rob_d, seed, sim_id)
            robot.generate_trajectory(os.path.join(trajectory_folder, filename + '.csv'))

    def heatmap(self, filename, stiff_values=None, min_num_mods=None,
                max_num_modules=None):
        """ Plot the archive as a heatmap.
            Note: current implementation deal only with 2 dimensions

        :param filename: the filename where to plot the archive
        :param stiff_values: a list of stiffness values to be uses as axis labels
        :param min_num_mods:
        :param max_num_modules:
        :return:
        """
        font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
        matplotlib.rc('font', **font)
        matplotlib.rcParams["axes.titlepad"] = 15
        matplotlib.rcParams['figure.dpi'] = 300

        parch = np.full(shape=self.values.shape, fill_value=np.NaN)
        for i in range(self.values.shape[0]):
            for j in range(self.values.shape[1]):
                if self.values[i, j] is not None:
                    parch[i, j] = self.values[i, j].fitness.values[0]

        fig = plt.figure(figsize=(12, 5.5))
        ax = fig.gca()
        ax.set_title('MAP-Elites Features Space', size=20, fontweight='normal')
        ax.set_xlabel('Stiffness value', labelpad=10)
        ax.set_ylabel('# modules', labelpad=10)

        im = plt.imshow(parch, cmap='viridis', origin='lower')

        im_ratio = parch.shape[0] / parch.shape[1]
        cbar = fig.colorbar(im, ax=ax, fraction=im_ratio * 0.047, pad=0.04)
        cbar.set_label('Fitness', rotation=270, labelpad=20)

        if stiff_values is not None:
            plt.xticks(range(len(stiff_values)), stiff_values)
        if max_num_modules is not None and min_num_mods is not None:
            plt.yticks(range(max_num_modules - min_num_mods + 1),
                       range(min_num_mods, max_num_modules + 1))
        plt.savefig(filename, bbox_inches='tight')

    def heatmap_3d(self, filename, stiff_values=None, min_num_mods=None,
                max_num_modules=None):
        """ Plot the archive as a heatmap.
            Note: current implementation deal only with 2 dimensions

        :param filename: the filename where to plot the archive
        :param stiff_values: a list of stiffness values to be uses as axis labels
        :param min_num_mods:
        :param max_num_modules:
        :return:
        """
        font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
        matplotlib.rc('font', **font)
        matplotlib.rcParams["axes.titlepad"] = 15
        matplotlib.rcParams['figure.dpi'] = 300

        d1 = max_num_modules - min_num_mods + 1
        d2 = d3 = len(stiff_values)

        arch = np.full(shape=(d2, d1 * d3), fill_value=np.NaN)
        for i, nmods in enumerate(self.values):
            for j, stiff_min in enumerate(nmods):
                for k, robot in enumerate(stiff_min):
                    if robot is not None:
                        arch[k, j*d1 + i] = robot.get_fitness()

        fig = plt.figure(figsize=(15, 7))
        ax = fig.gca()
        ax.set_title('MAP-Elites Features Space', size=20, fontweight='normal')
        ax.set_xlabel('Min Module Stiffness', labelpad=10)
        ax.set_ylabel('Max Module Stiffness', labelpad=10)

        # Major ticks
        ax.set_xticks(np.arange(-.5, d1 * d2, d1))
        ax.set_yticks(np.arange(0, d3, 1))

        # prepare custom ticks labels
        m_labels = []
        for st in stiff_values:
            for k in range(d1 // 2):
                m_labels.append('')
            m_labels.append(st)
            for k in range(d1 // 2):
                m_labels.append('')

        # Labels for major ticks
        ax.set_xticklabels(m_labels, minor=True)
        ax.set_yticklabels(stiff_values)

        # Minor ticks
        ax.set_xticks(np.arange(-.5, d1 * d2, 1), minor=True)
        ax.set_yticks(np.arange(-.5, d3, 1), minor=True)

        ax_top = ax.twiny()
        ax_top.set_xticks(np.arange(0, d1 * d2, 1), minor=True)
        ax_top.set_xticklabels(list(range(min_num_mods, max_num_modules+1)) * d2,
                               minor=True, fontsize=5, rotation=90)
        ax_top.set_xlabel('# Modules', labelpad=10)

        # fix zorder of first x axis
        ax.set_zorder(10)
        ax.patch.set_visible(False)

        # Gridlines based on minor ticks
        ax.grid(axis='x', which='minor', color='lightgray', linestyle='-', linewidth=0.25, alpha=0.4)
        ax.grid(axis='x', which='major', color='k', linestyle='-', linewidth=0.75)
        ax.grid(axis='y', which='minor', color='k', linestyle='-', linewidth=1)
        ax.tick_params(axis='x', which='minor', bottom=False, top=False, labelbottom=True, labeltop=False)
        ax.tick_params(axis='x', which='major', bottom=False, top=False, labelbottom=False)
        ax.tick_params(axis='y', which='both', left=False, right=False, labelleft=True)

        ax_top.tick_params(axis='x', which='minor', bottom=False, top=False, labelbottom=False, labeltop=True)
        ax_top.tick_params(axis='x', which='major', bottom=False, top=False, labeltop=False)

        im = plt.imshow(arch, cmap='viridis', aspect='auto', origin='lower')

        im_ratio = arch.shape[0] / arch.shape[1]
        cbar = fig.colorbar(im, ax=ax, fraction=im_ratio, pad=0.04)
        cbar.set_label('Fitness', rotation=270, labelpad=20)

        plt.savefig(filename, bbox_inches='tight')
        plt.close()

# ================================================================ #


def evaluate_sols(toolbox, solutions, archive, history, parents_list=None, use_curiosity=False, coev=False):
    """
        Evaluate given solutions and add them to the archive in case their fitness is better
        than their corresponding elite. In case curiosity is chosen, parent_list must be a list
        containing the parents that generated the evaluated offsprings, in the same order.

        :param toolbox:
        :param solutions:
        :param archive:
        :param history:     necessary to store and skip already computed solutions
        :param parents_list:
        :param use_curiosity:
        :param coev: perform co-evolution, which implies not skipping individuals based on history
    """
    evaluated_sols, skips, n_eval = evaluate_ind(toolbox, solutions, history, coev=coev)

    if use_curiosity:
        for i, sol in enumerate(evaluated_sols):
            is_added = archive.add(sol)

            # update curiosity score
            b = parents_list[i].features_descriptor()
            if is_added:
                archive.inc_curiosity(b)
            else:
                archive.dec_curiosity(b)
    else:
        for sol in evaluated_sols:
            archive.add(sol)

    return skips, n_eval


# ============================================ #
#             MAP-Elites Algorithm             #
# ============================================ #
def run(init_solutions, toolbox, evo_file, max_num_sims, archive_shape,
        batch_size, verbose=__debug__, cp_freq=5, arch_folder='',
        history_file=None, use_curiosity=False, seed=0,
        cp_folder=None, sim_id=None, alg_data=None, time_no_update=10000,
        coev=False, morphologies=None, hall_of_fame=None):
    """ Execute MAP-Elites algorithm with given configuration

    :param init_solutions: the number of solutions
    :param toolbox: DEAP instance for using EA operators
    :param evo_file: the file descriptor where are stored all the collected
                     information about the evolutionary process
    :param max_num_sims:
    :param archive_shape:
    :param batch_size:
    :param verbose:
    :param cp_freq:
    :param arch_folder:
    :param history_file:
    :param use_curiosity: whether the selection process should use curiosity mechanism
    :param seed:
    :param cp_folder:
    :param sim_id:
    :param alg_data:
    :param time_no_update: after how many simulations without best fitness update
                           the evolution should be stopped
    :param coev: perform co-evolution, which implies not skipping individuals based on history
    :param morphologies: morphology -> id map required for coevolution
    :param hall_of_fame: hall of fame used in coevolution
    :return:
    """

    # create archives folder
    curiosity_dir = os.path.join(arch_folder, 'curiosity')
    os.makedirs(os.path.join(arch_folder, 'archives'), exist_ok=True)
    os.makedirs(curiosity_dir, exist_ok=True)

    history = load_history(history_file)
    # generate the mapping behaviour-fitness
    archive = Archive(archive_shape)

    # progress bar
    pbar = tqdm(total=max_num_sims, postfix={})

    # ARCHIVE INITIALIZATION
    # evaluate initial solutions and add them to the archive
    num_init_sols = len(init_solutions)

    if alg_data is not None:
        num_sims = alg_data['last_num_sims']
        num_gen = alg_data['num_gen']
        archive.values = alg_data['archive']
        archive.curiosity = alg_data['curiosity']

        pbar.update(num_sims)
        skips = num_sims
        print('Restart evolution from {}'.format(num_sims))
    else:
        num_sims = 0

        # in any case here is not needed the use of curiosity - only testing random generated solutions
        skips, n_eval = evaluate_sols(toolbox, init_solutions, archive, history, coev=coev)
        pbar.update(min(n_eval, max_num_sims - num_sims))
        num_sims += n_eval
        num_gen = 0

        record_population(num_sims, archive.to_list(), evo_file, skips,
                          pbar=pbar, verbose=verbose, coev=coev)
        archive.record_curiosity(os.path.join(curiosity_dir,
                                              'curiosity_{}_{}.csv'.format(seed, num_sims)))
        if len(archive_shape) == 2:
            archive.write_map(os.path.join(arch_folder, 'archives',
                                           'archive_nsims_{}.csv'.format(num_sims)))
        else:
            archive.write_3d_map(os.path.join(arch_folder, 'archives',
                                             'archive_3d_nsims_{}.csv'.format(num_sims)))

    counter = 0
    best_fit = (max(archive.to_list(), key=lambda x: x.fitness.values)).fitness.values[0]

    while num_sims < max_num_sims and counter < time_no_update:
        # randomly generate a batch of new solutions
        # based on the current ones in the archive
        new_solutions = []

        parents = random.choices(archive.to_list(), k=batch_size,
                                 weights=archive.curiosity_probs())
        for parent in parents:
            offspring = toolbox.clone(parent)
            offspring, = toolbox.mutate(offspring)

            # remove fitness computed for the parent
            del offspring.fitness.values

            new_solutions.append(offspring)

        cskips, n_eval = evaluate_sols(toolbox, new_solutions, archive, history,
                                       parents, use_curiosity, coev=coev)
        skips += cskips
        pbar.update(min(n_eval, max_num_sims - num_sims))
        num_sims += n_eval
        num_gen += 1

        curr_best_fit = (max(new_solutions, key=lambda x: x.fitness.values)).fitness.values[0]
        if curr_best_fit > best_fit:
            best_fit = curr_best_fit
            counter = 0
        else:
            counter += batch_size

        if not coev and ((num_sims - num_init_sols) % (batch_size * cp_freq) == 0) or \
                coev and ((num_gen+1) % cp_freq == 0):
            evo_file.write(', ')
            record_population(num_sims, archive.to_list(), evo_file, skips,
                              pbar=pbar, verbose=verbose, coev=coev)
            archive.record_curiosity(os.path.join(curiosity_dir,
                                                  'curiosity_{}_{}.csv'.format(seed, num_sims)))

            store_checkpoint(
                {
                    'archive': archive.values,
                    'curiosity': archive.curiosity,
                    'num_sims': num_sims,
                    'num_gen': num_gen,
                    'rnd_state': random.getstate(),
                    'morphologies': morphologies,
                    'hall_of_fame': hall_of_fame
                },
                os.path.join(cp_folder, 'checkpoint_{}_{}.pkl'.format(seed, sim_id))
            )

            # save a temporary history that can be used
            # in case of execution interruptions
            if not coev:
                store_history(history, os.path.join(arch_folder, 'history_tmp.csv'))

            if len(archive_shape) == 2:
                archive.write_map(os.path.join(arch_folder, 'archives',
                                               'archive_nsims_{}.csv'.format(num_sims)))
            else:
                archive.write_3d_map(os.path.join(arch_folder, 'archives',
                                                  'archive_3d_nsims_{}.csv'.format(num_sims)))

    last_checkpoint = {
        'archive': archive.values,
        'curiosity': archive.curiosity,
        'num_sims': num_sims,
        'num_gen': num_gen,
        'rnd_state': random.getstate(),
        'morphologies': morphologies,
        'hall_of_fame': hall_of_fame
    }

    # save latest data
    store_checkpoint(
        last_checkpoint,
        os.path.join(cp_folder, 'checkpoint_{}_{}.pkl'.format(seed, sim_id))
    )

    pbar.close()

    return archive, history
