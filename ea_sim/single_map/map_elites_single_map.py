import os
import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import pandas as pd
import random
import seaborn as sns
from tqdm import tqdm

from double_map.map_elites_double_map import DR, evaluate_solutions, perform_mutation
from single_map.utils_single_map import record_population
from utils import store_checkpoint


class SingleArchive:
    def __init__(self, e_shape, nn_shape):
        self.shape = e_shape + nn_shape
        self.values = np.full(shape=self.shape, fill_value=None)

    def add(self, e_nn_pair, e_bd, nn_bd, fitness, sensory_data, maximize):
        bd = e_bd + nn_bd
        if self.values[bd] is None:
            self.values[bd] = (e_nn_pair, fitness, sensory_data)
        else:
            if maximize:
                is_better = fitness > self.values[bd][1]
            else:
                is_better = fitness < self.values[bd][1]

            if is_better:
                self.values[bd] = (e_nn_pair, fitness, sensory_data)

    def get_sensory_data(self):
        sensory_data = []
        for entry in self.values.flatten():
            if entry is not None:
                sensory_data.append(entry[2])

        return sensory_data

    def re_init_nn_values(self, new_nn_bds, maximize):
        tmp_map = self.values
        self.values = np.full(shape=self.shape, fill_value=None)

        index = 0
        for entry in tmp_map.flatten():
            if entry is not None:
                self.add(entry[0], entry[0][0].bd(), new_nn_bds[index], entry[1], entry[2], maximize)
                index += 1

    def to_list(self):
        ret_list = [entry for entry in self.values.flatten() if entry is not None]

        return ret_list

    def write_map(self, map_filename, e_section_dims_names, e_section_bd_vals_labels, entity_id_map, dr_model):
        with open(map_filename, 'w') as out_file:
            ordinal_nums = ['1st', '2nd', '3rd', '4th']
            header = ','.join([
                '{}_dim:{}'.format(
                    ordinal_nums[i],
                    e_section_dims_names[i].lower().replace(' ', '_').replace('#', 'num')
                )
                for i in range(0, len(e_section_dims_names))
            ])
            out_file.write('1st_dim_indx,2nd_dim_indx,3rd_dim_indx,4th_dim_indx,'
                           '{},3rd_dim_val,4th_dim_val,fitness,e_id,nn_id\n'.format(header))

            for i in range(0, self.shape[0]):
                for j in range(0, self.shape[1]):
                    for k in range(0, self.shape[2]):
                        for l in range(0, self.shape[3]):
                            entry = self.values[i][j][k][l]
                            if entry is not None:
                                (entity, nn), fitness, sensory_data = entry
                                e_bd = entity.bd()
                                nn_bd_vals = dr_model.predicted_vals([sensory_data])[0]
                                out_file.write('{},{},{},{},{},{},{:.4f},{:.4f},{:.4f},{},{}\n'.format(
                                    e_bd[0],
                                    e_bd[1],
                                    k,
                                    l,
                                    e_section_bd_vals_labels[0][e_bd[0]],
                                    e_section_bd_vals_labels[1][e_bd[1]],
                                    nn_bd_vals[0],
                                    nn_bd_vals[1],
                                    fitness,
                                    entity_id_map[entity.representation()],
                                    nn.id
                                ))

    def heatmap(self, filename, e_section_dims_names, e_section_bd_vals_labels, boundaries):
        # dimensionality
        d = self.shape

        # fitness values
        data = np.full(shape=self.shape, fill_value=np.NaN)
        for i in range(0, self.shape[0]):
            for j in range(0, self.shape[1]):
                for k in range(0, self.shape[2]):
                    for l in range(0, self.shape[3]):
                        entry = self.values[i][j][k][l]
                        if entry is not None:
                            data[i][j][k][l] = entry[1]

        # format data
        _data = np.transpose(data, axes=[1, 0, 2, 3])
        data = np.transpose(_data.reshape((d[1], d[0] * d[2], d[3])), axes=[0, 2, 1])\
                 .reshape((d[1] * d[3], d[0] * d[2]))

        # create subplots
        plt.subplots(figsize=(10, 10))

        # generate heatmaps
        df_data = pd.DataFrame(data)
        ax = sns.heatmap(df_data, mask=df_data.isnull(), annot=False,  # norm=log_norm, cbar_kws={"ticks": cbar_ticks},
                         fmt=".4f", annot_kws={'size': 10}, linewidths=.5, linecolor='grey', cmap="viridis",
                         xticklabels=False, yticklabels=False
        )

        # set title, invert axes and set axes labels
        ax.set_title('MAP-Elites Feature Space', size=24, y=1.04)
        ax.invert_yaxis()
        ax.set_xlabel(e_section_dims_names[0], size=14, labelpad=10)
        ax.set_ylabel(e_section_dims_names[1], size=14, labelpad=10)

        # set ticks
        x_ticks_pos = np.linspace(d[2]/2.0, d[0] * d[2] - d[2]/2.0, d[2])
        y_ticks_pos = np.linspace(d[3]/2.0, d[1] * d[3] - d[3]/2.0, d[3])

        ax.xaxis.set_major_locator(ticker.FixedLocator(x_ticks_pos))
        ax.xaxis.set_major_formatter(ticker.FixedFormatter(e_section_bd_vals_labels[0]))

        ax.yaxis.set_major_locator(ticker.FixedLocator(y_ticks_pos))
        ax.yaxis.set_major_formatter(ticker.FixedFormatter(e_section_bd_vals_labels[1]))

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

    def e_projection_heatmap(self, filename, e_archive_dims_names, e_archive_bd_vals_labels, best_function):
        font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
        matplotlib.rc('font', **font)
        matplotlib.rcParams["axes.titlepad"] = 15
        matplotlib.rcParams['figure.dpi'] = 300

        e_shape = self.shape[0:2]

        parch = np.full(shape=tuple(reversed(e_shape)), fill_value=np.NaN)
        for i in range(0, self.shape[0]):
            for j in range(0, self.shape[1]):
                for k in range(0, self.shape[2]):
                    for l in range(0, self.shape[3]):
                        if self.values[i, j, k, l] is not None:
                            if np.isnan(parch[j, i]):
                                parch[j, i] = self.values[i, j, k, l][1]
                            else:
                                parch[j, i] = best_function(parch[j, i], self.values[i, j, k, l][1])

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

    def nn_projection_heatmap(self, filename, boundaries, best_function):
        font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
        matplotlib.rc('font', **font)
        matplotlib.rcParams["axes.titlepad"] = 15
        matplotlib.rcParams['figure.dpi'] = 300

        nn_shape = self.shape[2:]

        parch = np.full(shape=tuple(reversed(nn_shape)), fill_value=np.NaN)
        for i in range(0, self.shape[0]):
            for j in range(0, self.shape[1]):
                for k in range(0, self.shape[2]):
                    for l in range(0, self.shape[3]):
                        if self.values[i, j, k, l] is not None:
                            if np.isnan(parch[l, k]):
                                parch[l, k] = self.values[i, j, k, l][1]
                            else:
                                parch[l, k] = best_function(parch[l, k], self.values[i, j, k, l][1])

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
            plt.xticks(np.linspace(-0.5, nn_shape[0]-0.5, nn_shape[0]+1),
                       ["{:.2f}".format(bound_val) for bound_val in boundaries[0]])
            plt.yticks(np.linspace(-0.5, nn_shape[1]-0.5, nn_shape[1]+1),
                       ["{:.2f}".format(bound_val) for bound_val in boundaries[1]])
        plt.savefig(filename, bbox_inches='tight')


def parents_selection(archive, batch_size):
    parent_entries = random.choices(archive.to_list(), k=int(batch_size))

    return parent_entries


# ============================================ #
#             MAP-Elites Algorithm             #
# ============================================ #
def run(init_solutions, toolbox, entity_section_shape, entity_section_dims_names, entity_section_bd_vals_labels,
        nn_section_shape, archive_evo_file, history_file, entity_id_map, hall_of_fame, hof_size,
        res_folder, cp_folder, max_num_sims, batch_size, mutation_weights, maximize_fit=False,
        eval_utilities=None, verbose=__debug__, cp_freq=5, seed=0, dr_update_gens=None,
        alg_data=None, time_no_update=10000):
    """ Execute MAP-Elites algorithm with given configuration
    :param maximize_fit:
    :param entity_section_dims_names:
    :param entity_section_bd_vals_labels:
    :param hof_size:
    :param dr_update_gens:
    :param mutation_weights:
    :param init_solutions: the number of solutions
    :param toolbox: DEAP instance for using EA operators
    :param archive_evo_file:
    :param history_file:
    :param max_num_sims:
    :param entity_section_shape:
    :param nn_section_shape:
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
    archive = SingleArchive(entity_section_shape, nn_section_shape)

    # DR object
    dr_model = DR(nn_section_shape)
    if dr_update_gens is None:
        dr_update_gens = [0, 50, 150, 350, 750]

    # progress bar
    pbar = tqdm(total=max_num_sims, postfix={})

    # best function
    best_function = max if maximize_fit else min

    # ARCHIVE INITIALIZATION
    # evaluate initial solutions and add them to the archive
    if alg_data is not None:
        num_sims = alg_data['last_num_sims']
        skips = alg_data['skips']
        num_gen = alg_data['num_gen']
        archive.values = alg_data['archive']
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

        record_population(num_sims, archive.to_list(), entity_id_map, archive_evo_file, skips,
                          pbar=pbar, verbose=verbose)

        archive.write_map(
            os.path.join(res_folder, 'archives', 'archive_{}_ngen_{}.csv'.format(seed, num_gen)),
            entity_section_dims_names, entity_section_bd_vals_labels, entity_id_map, dr_model
        )
        dr_model.write_boundaries(
            os.path.join(res_folder, 'archives', 'nn_section_bounds_{}_ngen_{}.json'.format(seed, num_gen))
        )

    counter = 0
    best_fit = (best_function(archive.to_list(), key=lambda x: x[1]))[1]

    while num_sims < max_num_sims and counter < time_no_update:
        # randomly generate a batch of new solutions
        # based on the current ones in the archives
        num_gen += 1
        new_solutions = []

        parent_entries = parents_selection(archive, batch_size)
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

        curr_best_fit = (best_function(archive.to_list(), key=lambda x: x[1]))[1]
        if curr_best_fit > best_fit:
            best_fit = curr_best_fit
            counter = 0
        else:
            counter += batch_size

        archive_evo_file.write(', ')
        record_population(num_sims, archive.to_list(), entity_id_map, archive_evo_file, skips,
                          pbar=pbar, verbose=verbose)

        archive.write_map(
            os.path.join(res_folder, 'archives', 'archive_{}_ngen_{}.csv'.format(seed, num_gen)),
            entity_section_dims_names, entity_section_bd_vals_labels, entity_id_map, dr_model
        )
        dr_model.write_boundaries(
            os.path.join(res_folder, 'archives', 'nn_section_bounds_{}_ngen_{}.json'.format(seed, num_gen))
        )

        if (num_gen+1) % cp_freq == 0:
            store_checkpoint(
                {
                    'archive': archive.values,
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
                'archive': archive.values,
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
        archive.heatmap(
            os.path.join(res_folder, 'heatmaps', 'heatmap_{}_gen_{}.pdf'.format(seed, num_gen)),
            entity_section_dims_names, entity_section_bd_vals_labels, dr_model.boundaries
        )

        archive.e_projection_heatmap(
            os.path.join(res_folder, 'heatmaps', 'entity_heatmap_{}_gen_{}.pdf'.format(seed, num_gen)),
            entity_section_dims_names, entity_section_bd_vals_labels, best_function
        )

        archive.nn_projection_heatmap(
            os.path.join(res_folder, 'heatmaps', 'nn_heatmap_{}_gen_{}.pdf'.format(seed, num_gen)),
            dr_model.boundaries,
            best_function
        )

    pbar.close()

    return archive

