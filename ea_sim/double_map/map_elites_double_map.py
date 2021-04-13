import json
import os
import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import random
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler
from tqdm import tqdm

from double_map.utils_double_map import record_population, update_hall_of_fame
from robot import SUS
from utils import store_checkpoint


class DoubleArchive:
    def __init__(self, e_shape, nn_shape, deep_grid, depth):
        self.e_values = np.full(shape=e_shape, fill_value=None)
        self.nn_values = np.full(shape=nn_shape, fill_value=None)
        self.nn_shape = nn_shape
        self.deep_grid = deep_grid
        self.depth = depth

    def add(self, e_nn_pair, e_bd, nn_bd, fitness, sensory_data, maximize):
        self.add_to_e_map(e_nn_pair, e_bd, fitness, maximize)
        self.add_to_nn_map(e_nn_pair, nn_bd, fitness, sensory_data, maximize)

    def add_to_e_map(self, e_nn_pair, e_bd, fitness, maximize):
        # update entity map
        if self.e_values[e_bd] is None:
            self.e_values[e_bd] = (e_nn_pair, fitness)
        else:
            if maximize:
                is_better = fitness > self.e_values[e_bd][1]
            else:
                is_better = fitness < self.e_values[e_bd][1]

            if is_better:
                self.e_values[e_bd] = (e_nn_pair, fitness)

    def add_to_nn_map(self, e_nn_pair, nn_bd, fitness, sensory_data, maximize):
        if self.deep_grid:
            self.add_to_nn_map_deep(e_nn_pair, nn_bd, fitness, sensory_data)
        else:
            self.add_to_nn_map_flat(e_nn_pair, nn_bd, fitness, sensory_data, maximize)

    def add_to_nn_map_deep(self, e_nn_pair, nn_bd, fitness, sensory_data):
        # update controllers map
        if self.nn_values[nn_bd] is None:
            self.nn_values[nn_bd] = [(e_nn_pair, fitness, sensory_data)]
        elif len(self.nn_values[nn_bd]) < self.depth:
            self.nn_values[nn_bd].append((e_nn_pair, fitness, sensory_data))
        else:
            random_index = random.randint(0, self.depth-1)
            self.nn_values[nn_bd][random_index] = (e_nn_pair, fitness, sensory_data)

    def add_to_nn_map_flat(self, e_nn_pair, nn_bd, fitness, sensory_data, maximize):
        # update controllers map
        if self.nn_values[nn_bd] is None:
            self.nn_values[nn_bd] = (e_nn_pair, fitness, sensory_data)
        else:
            if maximize:
                is_better = fitness > self.nn_values[nn_bd][1]
            else:
                is_better = fitness < self.nn_values[nn_bd][1]

            if is_better:
                self.nn_values[nn_bd] = (e_nn_pair, fitness, sensory_data)

    def get_sensory_data(self):
        sensory_data = []
        for entry in self.nn_values.flatten():
            if entry is not None:
                if self.deep_grid:
                    for element in entry:
                        sensory_data.append(element[2])
                else:
                    sensory_data.append(entry[2])

        return sensory_data

    def re_init_nn_values(self, new_nn_bds, maximize):
        tmp_map = self.nn_values
        self.nn_values = np.full(shape=self.nn_shape, fill_value=None)

        index = 0
        for entry in tmp_map.flatten():
            if entry is not None:
                if self.deep_grid:
                    for element in entry:
                        self.add_to_nn_map(element[0], new_nn_bds[index], element[1], element[2], maximize)
                        index += 1
                else:
                    self.add_to_nn_map(entry[0], new_nn_bds[index], entry[1], entry[2], maximize)
                    index += 1

    def to_list(self, type, flatten=True):
        """
            Return the values stored in the archive as a list of
            lists without considering the empty spaces
        :param flatten:
        :param type:
        :return:
        """
        ret_list = None

        if type == 0:
            ret_list = [e_entry for e_entry in self.e_values.flatten() if e_entry is not None]
        elif type == 1:
            ret_list = [nn_entry for nn_entry in self.nn_values.flatten() if nn_entry is not None]
            if self.deep_grid and flatten:
                ret_list = [element for sublist in ret_list for element in sublist]
        elif type == 2:
            e_list = [e_entry for e_entry in self.e_values.flatten() if e_entry is not None]
            nn_list = [nn_entry for nn_entry in self.nn_values.flatten() if nn_entry is not None]
            if self.deep_grid and flatten:
                nn_list = [element for sublist in nn_list for element in sublist]

            ret_list = e_list + nn_list

        return ret_list

    def write_maps(self, e_map_filename, e_archive_dims_names, e_archive_bd_vals_labels,
                   entity_id_map, nn_map_filename, dr_model, maximize_fit):
        # write entity map
        with open(e_map_filename, 'w') as out_file:
            ordinal_nums = ['1st', '2nd', '3rd', '4th']
            header = ','.join([
                '{}_dim:{}'.format(
                    ordinal_nums[i],
                    e_archive_dims_names[i].lower().replace(' ', '_').replace('#', 'num')
                )
                for i in range(0, len(e_archive_dims_names))
            ])
            out_file.write('1st_dim_indx,2nd_dim_indx,{},fitness,e_id,nn_id\n'.format(header))
            for row in self.e_values:
                for entry in row:
                    if entry is not None:
                        (entity, nn), fitness = entry
                        e_bd = entity.bd()
                        out_file.write('{},{},{},{},{:.4f},{},{}\n'.format(
                            e_bd[0],
                            e_bd[1],
                            e_archive_bd_vals_labels[0][e_bd[0]],
                            e_archive_bd_vals_labels[1][e_bd[1]],
                            fitness,
                            entity_id_map[entity.representation()],
                            nn.id
                        ))

        # write neural networks map
        with open(nn_map_filename, 'w') as out_file:
            out_file.write('1st_dim_indx,2nd_dim_indx,1st_dim_val,2nd_dim_val,fitness,e_id,nn_id\n')
            for i in range(0, self.nn_shape[0]):
                for j in range(0, self.nn_shape[1]):
                    entry = self.nn_values[i][j]
                    if entry is not None:
                        if self.deep_grid:
                            sorted_entry = sorted(entry, key=lambda x: x[1], reverse=maximize_fit)
                            for element in sorted_entry:
                                (entity, nn), fitness, sensory_data = element
                                nn_bd_vals = dr_model.predicted_vals([sensory_data])[0]
                                out_file.write('{},{},{:.4f},{:.4f},{:.4f},{},{}\n'.format(
                                    i,
                                    j,
                                    nn_bd_vals[0],
                                    nn_bd_vals[1],
                                    fitness,
                                    entity_id_map[entity.representation()],
                                    nn.id
                                ))
                        else:
                            (entity, nn), fitness, sensory_data = entry
                            nn_bd_vals = dr_model.predicted_vals([sensory_data])[0]
                            out_file.write('{},{},{:.4f},{:.4f},{:.4f},{},{}\n'.format(
                                i,
                                j,
                                nn_bd_vals[0],
                                nn_bd_vals[1],
                                fitness,
                                entity_id_map[entity.representation()],
                                nn.id
                            ))

    def e_heatmap(self, filename, e_archive_dims_names, e_archive_bd_vals_labels):
        """ Plot the archive as a heatmap.
            Note: current implementation deal only with 2 dimensions

        :param e_archive_dims_names:
        :param e_archive_bd_vals_labels:
        :param filename: the filename where to plot the archive
        :return:
        """

        font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
        matplotlib.rc('font', **font)
        matplotlib.rcParams["axes.titlepad"] = 15
        matplotlib.rcParams['figure.dpi'] = 300

        parch = np.full(shape=tuple(reversed(self.e_values.shape)), fill_value=np.NaN)
        for i in range(self.e_values.shape[0]):
            for j in range(self.e_values.shape[1]):
                if self.e_values[i, j] is not None:
                    parch[j, i] = self.e_values[i, j][1]

        fig = plt.figure(figsize=(12, 5.5))
        ax = fig.gca()
        ax.set_title('MAP-Elites Feature Space', size=20, fontweight='normal')
        ax.set_xlabel(e_archive_dims_names[0], labelpad=10)
        ax.set_ylabel(e_archive_dims_names[1], labelpad=10)

        im = plt.imshow(parch, cmap='viridis', origin='lower')

        im_ratio = parch.shape[0] / parch.shape[1]
        cbar = fig.colorbar(im, ax=ax, fraction=im_ratio * 0.047, pad=0.04)
        cbar.set_label('Fitness', rotation=270, labelpad=20)

        plt.xticks(range(len(e_archive_bd_vals_labels[0])), e_archive_bd_vals_labels[0])
        plt.yticks(range(len(e_archive_bd_vals_labels[1])), e_archive_bd_vals_labels[1])

        plt.savefig(filename, bbox_inches='tight')

    def nn_heatmap(self, filename, boundaries, best_function):
        """ Plot the archive as a heatmap.
            Note: current implementation deal only with 2 dimensions

        :param best_function:
        :param boundaries:
        :param filename: the filename where to plot the archive
        :return:
        """
        font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
        matplotlib.rc('font', **font)
        matplotlib.rcParams["axes.titlepad"] = 15
        matplotlib.rcParams['figure.dpi'] = 300

        parch = np.full(shape=tuple(reversed(self.nn_values.shape)), fill_value=np.NaN)
        for i in range(self.nn_values.shape[0]):
            for j in range(self.nn_values.shape[1]):
                if self.nn_values[i, j] is not None:
                    if self.deep_grid:
                        best_fit = (best_function(self.nn_values[i, j], key=lambda x: x[1]))[1]
                        parch[j, i] = best_fit
                    else:
                        parch[j, i] = self.nn_values[i, j][1]

        fig = plt.figure(figsize=(12, 5.5))
        ax = fig.gca()
        ax.set_title('MAP-Elites Feature Space', size=20, fontweight='normal')
        ax.set_xlabel('1st dimension', labelpad=10)
        ax.set_ylabel('2nd dimension', labelpad=10)

        im = plt.imshow(parch, cmap='viridis', origin='lower')

        im_ratio = parch.shape[0] / parch.shape[1]
        cbar = fig.colorbar(im, ax=ax, fraction=im_ratio * 0.047, pad=0.04)
        cbar.set_label('Fitness', rotation=270, labelpad=20)

        if boundaries is not None:
            plt.xticks(np.linspace(-0.5, self.nn_shape[0]-0.5, self.nn_shape[0]+1),
                       ["{:.2f}".format(bound_val) for bound_val in boundaries[0]])
            plt.yticks(np.linspace(-0.5, self.nn_shape[1]-0.5, self.nn_shape[1]+1),
                       ["{:.2f}".format(bound_val) for bound_val in boundaries[1]])
        plt.savefig(filename, bbox_inches='tight')


class DR:
    def __init__(self, nn_archive_shape):
        self.ss = None
        self.pca = None
        self.nn_archive_shape = nn_archive_shape
        self.dimensions = len(nn_archive_shape)
        self.boundaries = None

    def is_initialized(self):
        return self.ss is not None and self.pca is not None and self.boundaries is not None

    def train(self, training_data):
        self.ss = StandardScaler()
        self.pca = PCA(n_components=self.dimensions)

        pca_input = self.ss.fit_transform(training_data)
        pca_output = self.pca.fit_transform(pca_input)

        self.learn_boundaries(pca_output)

    def learn_boundaries(self, pca_output):
        min_max = [
            (
                min(pca_output[:, i]),
                max(pca_output[:, i])
            )
            for i in range(0, self.dimensions)
        ]
        self.boundaries = [
            np.linspace(
                min_val-0.5*(max_val-min_val)/(n_components-1),
                max_val+0.5*(max_val-min_val)/(n_components-1),
                n_components+1
            )
            for n_components, (min_val, max_val) in zip(self.nn_archive_shape, min_max)
        ]

    def update_boundaries(self, boundaries_to_update):
        for i, min_val, max_val in boundaries_to_update:
            width = self.boundaries[i][1] - self.boundaries[i][0]
            abs_min = min(min_val, self.boundaries[i][0]+0.5*width)
            abs_max = max(max_val, self.boundaries[i][-1]-0.5*width)

            n_components = self.nn_archive_shape[i]
            self.boundaries[i] = np.linspace(
                abs_min - 0.5 * (abs_max - abs_min) / (n_components - 1),
                abs_max + 0.5 * (abs_max - abs_min) / (n_components - 1),
                n_components + 1
            )

    def predict(self, data):
        pca_output = self.pca.transform(self.ss.transform(data))

        boundaries_to_update = [
            (i, min(pca_output[:, i]), max(pca_output[:, i]))
            for i in range(0, self.dimensions)
            if min(pca_output[:, i]) < self.boundaries[i][0] or max(pca_output[:, i]) > self.boundaries[i][-1]
        ]
        if len(boundaries_to_update) > 0:
            self.update_boundaries(boundaries_to_update)

        bds = [
            tuple([
                min(
                    int(np.floor((val - bound_vals[0]) / (bound_vals[1] - bound_vals[0]))),
                    len(bound_vals)-1
                )
                for val, bound_vals in zip(row, self.boundaries)
            ])
            for row in pca_output
        ]
        return bds, len(boundaries_to_update) > 0

    def predicted_vals(self, data):
        return self.pca.transform(self.ss.transform(data))

    def write_boundaries(self, filename):
        json_boundaries = [array.tolist() for array in self.boundaries]
        with open(filename, 'w+') as file:
            json.dump(json_boundaries, file)


def evaluate_solutions(toolbox, solutions, archive, dr_model, dr_update, num_gen,
                       eval_utilities, entity_id_map, hall_of_fame, hof_size, history, maximize_fit):
    if dr_model.is_initialized() and dr_update:
        archive_sensory_data = archive.get_sensory_data()
        a_sd_df = pd.DataFrame(archive_sensory_data)

        dr_model.train(a_sd_df)
        nn_new_bds, bounds_update = dr_model.predict(a_sd_df)

        assert bounds_update is False
        archive.re_init_nn_values(nn_new_bds, maximize_fit)

    to_evaluate = []
    e_ids = []
    for i in range(0, len(solutions)):
        e, nn = solutions[i]
        representation = e.representation()

        if representation not in entity_id_map:
            e_id = max(entity_id_map.values()) + 1
            entity_id_map[representation] = e_id
        else:
            e_id = entity_id_map[representation]

        to_evaluate.append((
            e,
            e_id,
            nn,
            nn.id,
            random.randint(0, 1000000),
            eval_utilities
        ))
        e_ids.append(e_id)

    results = toolbox.map(toolbox.evaluate, to_evaluate)
    filtered_solutions, filtered_e_ids, filtered_results = [
        list(t)
        for t in zip(*[
            (sol, e_id, res)
            for sol, e_id, res in zip(solutions, e_ids, results)
            if res[0] is not None and res[1] is not None
        ])
    ]

    sensory_data = list(list(zip(*filtered_results))[1])
    sd_df = pd.DataFrame(sensory_data)

    if not dr_model.is_initialized() and dr_update:
        dr_model.train(sd_df)

    nn_bds, bounds_update = dr_model.predict(sd_df)

    if bounds_update:
        archive_sensory_data = archive.get_sensory_data()
        a_sd_df = pd.DataFrame(archive_sensory_data)

        nn_new_bds, bounds_update = dr_model.predict(a_sd_df)

        assert bounds_update is False
        archive.re_init_nn_values(nn_new_bds, maximize_fit)

    for sol, e_id, (fitness, sens_data), nn_bd in zip(filtered_solutions, filtered_e_ids, filtered_results, nn_bds):
        archive.add(sol, sol[0].bd(), nn_bd, fitness, sens_data, maximize_fit)

        history.write('{},{},{},{:.4f}\n'.format(num_gen, e_id, sol[1].id, fitness))
        update_hall_of_fame(hall_of_fame, hof_size, e_id, sol[1].id, fitness, maximize_fit)

    return len(results)-len(filtered_results), len(filtered_results)


def fitness_proportional_selection(fit_values, maximize_fit):
    fit_values_array = np.array(fit_values)

    if min(fit_values_array) < 1.0:
        fit_values_array = fit_values_array - min(fit_values_array) + 1.0

    if not maximize_fit:
        fit_values_array = 1.0 / fit_values_array

    r = random.uniform(0, sum(fit_values_array))

    incremental_sum = 0
    for idx, fit in enumerate(fit_values_array):
        incremental_sum += fit
        if incremental_sum >= r:
            return idx


def parents_selection(archive, batch_size, deep_grid, maximize_fit):
    e_entries = random.choices(archive.to_list(type=0), k=int(batch_size / 2))
    nn_entries = random.choices(archive.to_list(type=1, flatten=False), k=int(batch_size / 2))

    if deep_grid:
        selected_entries = []
        for entry in nn_entries:
            fit_vals = [element[1] for element in entry]
            selected_indx = fitness_proportional_selection(fit_vals, maximize_fit)
            selected_entries.append(entry[selected_indx])

        nn_entries = selected_entries

    parent_entries = e_entries + nn_entries
    random.shuffle(parent_entries)

    return parent_entries


def perform_mutation(tbox, parent, weights):
    offspring_entity, offspring_nn = tbox.clone(parent[0]), tbox.clone(parent[1])
    mutation_type = SUS(weights)[0]

    if mutation_type == 0:
        offspring_entity, = tbox.mutate_entity(offspring_entity)
    elif mutation_type == 1:
        offspring_nn = tbox.mutate_nn_genome(offspring_nn)
    else:
        offspring_entity, = tbox.mutate_entity(offspring_entity)
        offspring_nn = tbox.mutate_nn_genome(offspring_nn)

    offspring = (offspring_entity, offspring_nn)

    if offspring[0].representation() == parent[0].representation() and offspring[1].id == parent[1].id:
        return perform_mutation(tbox, parent, weights)

    return offspring


# ============================================ #
#             MAP-Elites Algorithm             #
# ============================================ #
def run(init_solutions, toolbox, entity_archive_shape, entity_archive_dims_names, entity_archive_bd_vals_labels,
        nn_archive_shape, entity_archive_evo_file, nn_archive_evo_file, history_file,
        entity_id_map, hall_of_fame, hof_size, res_folder, cp_folder,
        max_num_sims, batch_size, mutation_weights, maximize_fit=False, deep_grid=False, depth=50, log_all=False,
        eval_utilities=None, verbose=__debug__, cp_freq=5, seed=0, dr_update_gens=None,
        alg_data=None, time_no_update=10000):
    """ Execute MAP-Elites algorithm with given configuration
    :param log_all:
    :param maximize_fit:
    :param depth:
    :param deep_grid:
    :param entity_archive_dims_names:
    :param entity_archive_bd_vals_labels:
    :param hof_size:
    :param dr_update_gens:
    :param mutation_weights:
    :param init_solutions: the number of solutions
    :param toolbox: DEAP instance for using EA operators
    :param entity_archive_evo_file: the file descriptor where are stored all the collected
                           information about the entity archive evolutionary process
    :param nn_archive_evo_file: the file descriptor where are stored all the collected
                           information about the neural network archive evolutionary process
    :param history_file:
    :param max_num_sims:
    :param entity_archive_shape:
    :param nn_archive_shape:
    :param batch_size:
    :param eval_utilities:
    :param verbose:
    :param cp_freq:
    :param res_folder:
    :param seed:
    :param cp_folder:
    :param alg_data:
    :param time_no_update: after how many simulations without best fitness update
                           the evolution should be stopped
    :param entity_id_map: entity -> id map
    :param hall_of_fame: hall of fame
    :return:
    """

    # create archives folder
    os.makedirs(os.path.join(res_folder, 'archives'), exist_ok=True)

    # create heatmaps folder
    os.makedirs(os.path.join(res_folder, 'heatmaps'), exist_ok=True)

    # generate the behaviour-fitness maps
    archive = DoubleArchive(entity_archive_shape, nn_archive_shape, deep_grid, depth)

    # DR object
    dr_model = DR(nn_archive_shape)
    if dr_update_gens is None:
        dr_update_gens = [0, 50, 150, 350, 750]

    # progress bar
    pbar = tqdm(total=max_num_sims, postfix={})

    # best function
    best_function = max if maximize_fit else min

    # partial log for Deep Grid archive
    dg_partial_log = deep_grid and not log_all

    # ARCHIVE INITIALIZATION
    # evaluate initial solutions and add them to the archive
    if alg_data is not None:
        num_sims = alg_data['last_num_sims']
        skips = alg_data['skips']
        num_gen = alg_data['num_gen']
        archive.e_values = alg_data['e_archive']
        archive.nn_values = alg_data['nn_archive']
        dr_model = alg_data['dr_model']

        pbar.update(num_sims)
        print('Restart evolution from {}'.format(num_sims))
    else:
        num_gen = 0
        num_sims = 0

        skips, n_eval = evaluate_solutions(toolbox, init_solutions, archive, dr_model, num_gen in dr_update_gens,
                                           num_gen, eval_utilities, entity_id_map, hall_of_fame, hof_size,
                                           history_file, maximize_fit)
        pbar.update(min(n_eval, max_num_sims - num_sims))
        num_sims += n_eval

        record_population(num_sims, archive.to_list(type=0), entity_id_map, 0, entity_archive_evo_file,
                          skips, pbar=pbar, verbose=verbose)
        record_population(num_sims, archive.to_list(type=1, flatten=log_all), entity_id_map, 1, nn_archive_evo_file,
                          skips, pbar=None, verbose=False, dg_partial_log=dg_partial_log, best_function=best_function)

        archive.write_maps(
            os.path.join(res_folder, 'archives', 'entity_archive_{}_ngen_{}.csv'.format(seed, num_gen)),
            entity_archive_dims_names, entity_archive_bd_vals_labels, entity_id_map,
            os.path.join(res_folder, 'archives', 'nn_archive_{}_ngen_{}.csv'.format(seed, num_gen)),
            dr_model, maximize_fit
        )
        dr_model.write_boundaries(
            os.path.join(res_folder, 'archives', 'nn_archive_bounds_{}_ngen_{}.json'.format(seed, num_gen))
        )

    counter = 0
    best_fit = (best_function(archive.to_list(type=2), key=lambda x: x[1]))[1]

    while num_sims < max_num_sims and counter < time_no_update:
        # randomly generate a batch of new solutions
        # based on the current ones in the archives
        num_gen += 1
        new_solutions = []

        parent_entries = parents_selection(archive, batch_size, deep_grid, maximize_fit)
        for parent_entry in parent_entries:
            parent = parent_entry[0]
            offspring = perform_mutation(toolbox, parent, mutation_weights)
            new_solutions.append(offspring)

        cskips, n_eval = evaluate_solutions(toolbox, new_solutions, archive, dr_model, num_gen in dr_update_gens,
                                            num_gen, eval_utilities, entity_id_map,
                                            hall_of_fame, hof_size, history_file, maximize_fit)
        skips += cskips
        pbar.update(min(n_eval, max_num_sims - num_sims))
        num_sims += n_eval

        curr_best_fit = (best_function(archive.to_list(type=2), key=lambda x: x[1]))[1]
        if curr_best_fit > best_fit:
            best_fit = curr_best_fit
            counter = 0
        else:
            counter += batch_size

        entity_archive_evo_file.write(', ')
        nn_archive_evo_file.write(', ')
        record_population(num_sims, archive.to_list(type=0), entity_id_map, 0, entity_archive_evo_file,
                          skips, pbar=pbar, verbose=verbose)
        record_population(num_sims, archive.to_list(type=1, flatten=log_all), entity_id_map, 1, nn_archive_evo_file,
                          skips, pbar=pbar, verbose=False, dg_partial_log=dg_partial_log, best_function=best_function)

        archive.write_maps(
            os.path.join(res_folder, 'archives', 'entity_archive_{}_ngen_{}.csv'.format(seed, num_gen)),
            entity_archive_dims_names, entity_archive_bd_vals_labels, entity_id_map,
            os.path.join(res_folder, 'archives', 'nn_archive_{}_ngen_{}.csv'.format(seed, num_gen)),
            dr_model, maximize_fit
        )
        dr_model.write_boundaries(
            os.path.join(res_folder, 'archives', 'nn_archive_bounds_{}_ngen_{}.json'.format(seed, num_gen))
        )

        if (num_gen+1) % cp_freq == 0:
            store_checkpoint(
                {
                    'e_archive': archive.e_values,
                    'nn_archive': archive.nn_values,
                    'dr_model': dr_model,
                    'num_sims': num_sims,
                    'skips': skips,
                    'num_gen': num_gen,
                    'last_nn_ids': toolbox.last_nn_ids(),
                    'entity_id_map': entity_id_map,
                    'hall_of_fame': hall_of_fame,
                    'rnd_state': random.getstate(),
                },
                os.path.join(cp_folder, 'checkpoint_{}.pkl'.format(seed))
            )

    if alg_data is None or num_gen != alg_data['num_gen']:
        # save latest data
        store_checkpoint(
            {
                'e_archive': archive.e_values,
                'nn_archive': archive.nn_values,
                'dr_model': dr_model,
                'num_sims': num_sims,
                'skips': skips,
                'num_gen': num_gen,
                'last_nn_ids': toolbox.last_nn_ids(),
                'entity_id_map': entity_id_map,
                'hall_of_fame': hall_of_fame,
                'rnd_state': random.getstate(),
            },
            os.path.join(cp_folder, 'checkpoint_{}.pkl'.format(seed))
        )

        # generate heatmaps
        archive.e_heatmap(
            os.path.join(res_folder, 'heatmaps', 'entity_heatmap_{}_gen_{}.pdf'.format(seed, num_gen)),
            entity_archive_dims_names, entity_archive_bd_vals_labels
        )
        archive.nn_heatmap(
            os.path.join(res_folder, 'heatmaps', 'nn_heatmap_{}_gen_{}.pdf'.format(seed, num_gen)),
            dr_model.boundaries,
            best_function
        )

    pbar.close()

    return archive

