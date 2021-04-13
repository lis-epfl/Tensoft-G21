import os
import json
import pickle
import sqlite3
import numpy as np

import matplotlib
# select matplotlib backend
matplotlib.use('pdf')
import matplotlib.pyplot as plt

from params_conf import N_MODULES, STIFF_TABLE


# ================================== #
#           Utils functions          #
# ================================== #
def print_dict(d, level=0, list_on_levels=False):
    for k, v in sorted(d.items()):
        if isinstance(v, dict):
            print('\t'*level + k+':')
            print_dict(v, level+1, list_on_levels)
        elif isinstance(v, list) and list_on_levels and k == 'modules_conf':
            print('\t' * level + '{}: ['.format(k))
            for element in v:
                print('\t' * (level+1) + '{}'.format(element))
            print('\t' * level + ']')
        else:
            print('\t'*level + '{}: {}'.format(k, v))


def print_header(config, bar_num=50):
    print('# ' + '='*bar_num + ' #')
    print('\t\tEVOLUTIONARY SIMULATION\n\t' + ' '*7 + 'OF TENSEGRITY SOFT ROBOTS')
    print('# ' + '='*bar_num + ' #\n')

    print('# ' + '='*bar_num + ' #')
    print('\t' + ' '*7 + 'EXPERIMENT  CONFIGURATION')
    print('# ' + '='*bar_num + ' #')
    print_dict(config)
    print('# ' + '='*bar_num + ' #')


def print_header_contr_evo(config, bar_num=50):
    print('# ' + '=' * bar_num + ' #')
    print('\t\tEVOLUTIONARY SIMULATION\n\t' +
          ' OF TENSEGRITY SOFT ROBOTS CONTROLLERS \n\t' +
          ('FOR GOAL REACHING AFTER SQUEEZING TASK' if 'SGR' in config['simulation_path']
           else ' ' * 8 + 'FOR GOAL REACHING TASK')
    )
    print('# ' + '=' * bar_num + ' #\n')

    print('# ' + '=' * bar_num + ' #')
    print('\t' + ' ' * 7 + 'EXPERIMENT  CONFIGURATION')
    print('# ' + '=' * bar_num + ' #')
    print_dict(config, list_on_levels=True)
    print('# ' + '=' * bar_num + ' #')


def print_header_coev(config, bar_num=55):
    print('# ' + '=' * bar_num + ' #')
    print('\t\tCO-EVOLUTIONARY SIMULATION\n\t' +
          ' ' * 7 + 'OF MORPHOLOGY AND CONTROLLER\n\t' +
          ' ' * 8 + 'OF TENSEGRITY SOFT ROBOTS\n\t' +
          (' ' * 2 + 'FOR GOAL REACHING AFTER SQUEEZING TASK' if 'SGR' in config['simulation_path']
           else ' ' * 10 + 'FOR GOAL REACHING TASK')
    )
    print('# ' + '=' * bar_num + ' #\n')

    print('# ' + '=' * bar_num + ' #')
    print('\t' + ' ' * 8 + 'EXPERIMENT  CONFIGURATION')
    print('# ' + '=' * bar_num + ' #')
    print_dict(config, list_on_levels=False)
    print('# ' + '=' * bar_num + ' #')


def print_header_double_map(config, bar_num=55):
    print('# ' + '=' * bar_num + ' #')
    print('\t\t  EVOLUTIONARY SIMULATION\n\t' +
          ' ' * 7 + 'OF MORPHOLOGY AND CONTROLLER\n\t' +
          ' ' * 8 + 'OF TENSEGRITY SOFT ROBOTS\n\t' +
          (' ' * 2 + 'FOR GOAL REACHING AFTER SQUEEZING TASK' if 'SGR' in config['simulation_path']
           else ' ' * 10 + 'FOR GOAL REACHING TASK') + '\n\t' +
          ' ' * 7 + '(DOUBLE MAP-ELITES VARIANT)'
    )
    print('# ' + '=' * bar_num + ' #\n')

    print('# ' + '=' * bar_num + ' #')
    print('\t' + ' ' * 8 + 'EXPERIMENT  CONFIGURATION')
    print('# ' + '=' * bar_num + ' #')
    print_dict(config, list_on_levels=False)
    print('# ' + '=' * bar_num + ' #')


def print_header_single_map(config, bar_num=55):
    print('# ' + '=' * bar_num + ' #')
    print('\t\t  EVOLUTIONARY SIMULATION\n\t' +
          ' ' * 7 + 'OF MORPHOLOGY AND CONTROLLER\n\t' +
          ' ' * 8 + 'OF TENSEGRITY SOFT ROBOTS\n\t' +
          (' ' * 2 + 'FOR GOAL REACHING AFTER SQUEEZING TASK' if 'SGR' in config['simulation_path']
           else ' ' * 10 + 'FOR GOAL REACHING TASK') + '\n\t' +
          ' ' * 7 + '(SINGLE MAP-ELITES VARIANT)'
    )
    print('# ' + '=' * bar_num + ' #\n')

    print('# ' + '=' * bar_num + ' #')
    print('\t' + ' ' * 8 + 'EXPERIMENT  CONFIGURATION')
    print('# ' + '=' * bar_num + ' #')
    print_dict(config, list_on_levels=False)
    print('# ' + '=' * bar_num + ' #')


def store_checkpoint(checkpoint, filename):
    with open(filename, 'wb') as cp_file:
        pickle.dump(checkpoint, cp_file)


def record_info(logbook, stats, gen, pop, inv_ind):
    if stats is not None:
        record = stats.compile(pop) if stats is not None else {}
        logbook.record(gen=gen, nevals=len(inv_ind), **record)


def record_population(num_sims, population, file, skips, pbar=None, verbose=False, coev=False):
    fitness_values = np.asarray(list(map(lambda i: i.fitness.values, population)))
    if coev:
        fitness_values = [1.0 / f for f in fitness_values]

    pop_stats = {
        'num_sims': num_sims,
        'avg_fitness': np.mean(fitness_values),
        'std_dev': np.std(fitness_values),
        'min': np.min(fitness_values),
        'max': np.max(fitness_values)
    }

    # store the current population values
    file.write(json.dumps({
        **pop_stats,
        'population': [individual.info(coev=coev) for individual in population]
    }))

    if verbose:
        if pbar is not None:
            pbar.set_postfix({
                'avg': pop_stats['avg_fitness'],
                'std': pop_stats['std_dev'],
                'min': pop_stats['min'],
                'max': pop_stats['max'],
                'skip': skips
            })
        else:
            print('num_sims: {} | Fitness -> avg: {} std: {} min: {} max: {}'.format(
                pop_stats['num_sims'], pop_stats['avg_fitness'],
                pop_stats['std_dev'], pop_stats['min'], pop_stats['max']
            ))


def evaluate_ind(toolbox, individuals, glob_history, local_history=None, coev=False, eval_all=False):
    """

    :param toolbox:
    :param individuals:
    :param glob_history: a dictionary that maps simulator
                         input strings into corresponding computed fitness.
    :param local_history:
    :param coev:
    :param eval_all:
    :return:
    """
    # consider only new individuals (offsprings)
    invalid_ind = [ind for ind in individuals if (not ind.fitness.valid or (coev and eval_all))]

    # select for evaluation only solutions
    # that have not been already evaluated
    skip = []
    to_evaluate = []

    for ind in invalid_ind:
        ind_string = ind.string_input()
        if not coev and ind_string in glob_history:
            # assign fitness previously computed for the same configuration
            ind.fitness.values = glob_history[ind_string]
            # store also record into a local history to support MAP creation functionality
            if local_history is not None:
                local_history[ind_string] = glob_history[ind_string]

            skip.append(ind)
        else:
            to_evaluate.append(ind)

    n_evaluations = 0
    if len(to_evaluate) > 0:
        fitnesses = toolbox.map(toolbox.evaluate, to_evaluate)
        n_evaluations = len(fitnesses)
        for ind, fit in zip(to_evaluate, fitnesses):
            ind.fitness.values = fit
            ind_string = ind.string_input()
            # update history records with latest fitness
            glob_history[ind_string] = fit

            # store also record into a local history to support MAP creation functionality
            if local_history is not None:
                local_history[ind_string] = fit

    if not coev:
        n_evaluations = len(individuals)

    return to_evaluate + skip, len(skip), n_evaluations


def plot_population_stats(pop, results_folder, seed, num_sim, normalize=False):
    """ Plot the fitness/num_modules/stiffness distribution across given robot population

    :param pop: population of robot individuals
    :param results_folder: folder where to store the plots
    :param seed: simulation seed
    :param num_sim: simulation id
    :param normalize: whether to reports the plots in a normalized manner (default: False)
    :return:
    """
    # split robots interested properties into three lists
    fits, n_mods, stiffs = list(zip(*[(ind.get_fitness(), ind.num_modules, ind.stiffness)
                                      for ind in pop]))

    # plot the properties distribution of last generation
    configs = [
        {
            'data': fits,
            'title': 'Fitness',
            'h_range': (0, np.max(fits)),
            'out_file': os.path.join(results_folder, 'fitness',
                                     'fit_dist_sim_{}_{}.pdf'.format(seed, num_sim)),
            'bins': max(len(fits)//4, 1),
            'discrete_hist': False,
            'norm': normalize
        },
        {
            'data': n_mods,
            'title': '# modules',
            'h_range': (2, 11),
            'out_file': os.path.join(results_folder, 'n_modules',
                                     'num_mods_dist_sim_{}_{}.pdf'.format(seed, num_sim)),
            'bins': N_MODULES,
            'discrete_hist': True,
            'norm': normalize
        },
        {
            'data': stiffs,
            'title': 'Stiffness',
            'out_file': os.path.join(results_folder, 'stiffness',
                                     'stiff_dist_sim_{}_{}.pdf'.format(seed, num_sim)),
            'bins': STIFF_TABLE,
            'discrete_hist': True,
            'norm': normalize
        }
    ]

    for conf in configs:
        plot_population_dist(conf)


def plot_population_dist(conf):
    """ Plot the distribution of the values of given property across

    :param conf:
    :return:
    """
    font = {'family': 'Source Sans Pro', 'size': 12, 'weight': 'light'}
    matplotlib.rc('font', **font)
    matplotlib.rcParams["axes.titlepad"] = 15
    matplotlib.rcParams['figure.dpi'] = 300
    # create potential needed directory where to store the graph
    os.makedirs(os.path.dirname(conf['out_file']), exist_ok=True)

    num_colors = len(conf['bins']) if isinstance(conf['bins'], list) else conf['bins']
    colors = plt.cm.viridis(np.linspace(0, 1, num_colors))

    fig = plt.figure(figsize=(12, 5))
    ax = fig.gca()

    if conf['discrete_hist']:
        inds_vals = {str(sv): 0 for sv in conf['bins']}
        for ind_stiff in conf['data']:
            inds_vals[str(ind_stiff)] += 1
        x, y = list(zip(*[(k, v) for k, v in sorted(inds_vals.items(), key=lambda r: float(r[0]))]))
        if conf['norm']:
            y = np.asarray(y)/np.sum(y)
        ax.bar(x, y, color=colors)
    else:
        _, bins, patches = ax.hist(conf['data'], bins=conf['bins'], density=conf['norm'],
                             range=conf['h_range'])
        ax.set_xticks(bins)
        for i, (c, p) in enumerate(zip(bins, patches)):
            plt.setp(p, 'facecolor', colors[i])

    ax.set_title('{} distribution across last generation'.format(conf['title']),
                 fontweight='normal')
    ax.set_xlabel(conf['title'])
    ax.set_ylabel('# Individuals')
    if conf['norm']:
        ax.set_ylim(0, 1.1)

    plt.savefig(conf['out_file'], bbox_inches='tight')
    plt.close()


def parse_robot_string(rb_string):
    robot = []
    for module_str in rb_string.split('--')[:-1]:
        params = module_str.split('-')
        module = {
            'order': int(params[0].strip()),
            'connectedModules': int(params[1].strip()),
            'connectedFaces': int(params[2].strip()),
            'freq': float(params[3].strip()),
            'amplitude': float(params[4].strip()),
            'phase': float(params[5].strip()),
            'rot': float(params[6].strip()),
            'stiff': float(params[7].strip())
        }
        robot.append(module)
    return robot


# ================================== #
#       SIM HISTORY MANAGEMENT       #
# ================================== #

def load_history(history_file):
    """ Load simulation history from provided file.
    In case no file is given, returns an empty history

    :param history_file:
    :return:
    """
    history = {}
    if history_file is not None and os.path.exists(history_file):
        with open(history_file) as history_in:
            # skip header
            history_in.readline()
            for line in history_in:
                robot_string, fitness = line.strip().split(',')
                history[robot_string.strip()] = (float(fitness),)

    return history

def store_history(history, history_file):
    """ Store simulation history in provided file

    :param history:
    :param history_file:
    :return:
    """
    with open(history_file, 'w') as out_file:
        out_file.write('rob_string,fitness\n')
        for rob_string, fit in history.items():
            out_file.write('{},{}\n'.format(rob_string.strip(), fit[0]))


# NOTE: these functions below are currently not used.
def load_history_db(history_db):
    """ Load simulation history from provided db.
    In case no db is given, returns an empty history

    :param history_db:
    :return:
    """
    history = {}
    if history_db is not None:
        conn = sqlite3.connect(history_db)
        cursor = conn.cursor()

        for robot_string, fitness in cursor.execute('SELECT * FROM history'):
            history[robot_string] = (float(fitness),)

        cursor.close()
        conn.close()

    return history


def store_history_db(history, history_db):
    """ Store simulation history in provided db

    :param history:
    :param history_db:
    :return:
    """

    def history_gen():
        for record in history.items():
            yield record

    to_init = not os.path.exists(history_db)
    conn = sqlite3.connect(history_db)
    cursor = conn.cursor()

    # create the
    if to_init:
        cursor.execute('''CREATE TABLE history(robot_string VARCHAR PRIMARY KEY, fitness REAL NOT NULL)''')

    cursor.executemany('''REPLACE INTO history(robot_string, fitness) VALUES (?)''', history_gen())

    cursor.close()
    conn.close()