import os
import random

from tqdm import tqdm
from utils import (store_checkpoint, record_population,
                   evaluate_ind, load_history, store_history)


def run(population, toolbox, evo_file, mu, lambda_,
        cross_prob, mut_prob, max_num_sims, verbose=__debug__,
        cp_folder='', cp_freq=20, history_file=None,
        sim_id=None, seed=0, alg_data=None, time_no_update=10000,
        coev=False, eval_all=False, morphologies=None, hall_of_fame=None):
    """ Execute the Mu+Lambda algorithm.

    :param population: list of evolved individuals
    :param toolbox: DEAP instance for using EA operators
    :param evo_file: the file descriptor where are stored all the collected
                     information about the evolutionary process
    :param mu: the mu parameter of the algorithm
    :param lambda_: the lambda parameter of the algorithm
    :param cross_prob:
    :param mut_prob:
    :param max_num_sims:
    :param verbose: boolean flag to select whether progression output should be displayed
    :param cp_folder: path where checkpoint files are stored
    :param cp_freq: the number of generations between two checkpoints
    :param history_file: path where the potential history file is located
    :param sim_id: the identifier of current experiment run
    :param seed: the seed used to initialize the random generator
    :param alg_data: the algorithm data extracted by a checkpoint file
                     and necessary to reinitialize the state of the system.
                     By default no checkpoint is exploited.
    :param time_no_update: after how many simulations without best fitness update
                           the evolution should be stopped
    :param coev: perform co-evolution, which implies not skipping individuals based on history
    :param eval_all: evaluate all individuals at every generation, it is used in combination with coev
    :param morphologies: morphology -> id map required for coevolution
    :param hall_of_fame: hall of fame used in coevolution
    :return: the last population, with the best solution from each family
             and generated simulation history
    """

    history = load_history(history_file)
    local_history = {}
    pbar = tqdm(total=max_num_sims, postfix={})

    if alg_data is not None:
        # restore previous checkpoint
        num_sims = alg_data['last_num_sims']
        num_gen = alg_data['num_gen']
        skips = num_sims
        pbar.update(num_sims)
        print('Restart evolution from {}'.format(num_sims))
    else:
        num_sims = 0

        # Evaluate the individuals with an invalid fitness
        _, skips, n_eval = evaluate_ind(toolbox, population, history,
                                        local_history, coev=coev, eval_all=eval_all)
        pbar.update(min(n_eval, max_num_sims - num_sims))
        num_sims += n_eval
        num_gen = 0

        record_population(num_sims, population, evo_file, skips,
                          pbar=pbar, verbose=verbose, coev=coev)

    counter = 0
    best_fit = (max(population, key=lambda x: x.fitness.values)).fitness.values[0]

    # Begin the evolution process
    while num_sims < max_num_sims and counter < time_no_update:
        # write a comma to separate generation data in log file
        evo_file.write(', ')

        # Breed the lambda individuals
        offspring = var_or(population, toolbox, lambda_, cross_prob, mut_prob)

        if coev and eval_all:
            evaluated, cskips, n_eval = evaluate_ind(toolbox, population + offspring, history,
                                                     local_history, coev=coev, eval_all=eval_all)
            population = evaluated
        else:
            # Evaluate the newly generated individuals
            evaluated, cskips, n_eval = evaluate_ind(toolbox, offspring, history,
                                                     local_history, coev=coev, eval_all=eval_all)
            population += evaluated
        skips += cskips
        pbar.update(min(n_eval, max_num_sims - num_sims))
        num_sims += n_eval
        num_gen += 1

        # Select the next generation population
        population[:] = toolbox.selectBest(population, mu)

        # Store all information of individuals of current generation
        record_population(num_sims, population, evo_file, skips,
                          pbar=pbar, verbose=verbose, coev=coev)

        if not coev and (num_sims % (cp_freq*mu) == 0) or \
                coev and ((num_gen+1) % cp_freq == 0):
            store_checkpoint(
                {
                    'population': population,
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
                store_history(history, os.path.join(cp_folder, 'history_tmp.csv'))

        # termination criteria based on fitness aging
        curr_best_fit = (max(population, key=lambda x: x.fitness.values)).fitness.values[0]
        if curr_best_fit > best_fit:
            best_fit = curr_best_fit
            counter = 0
        else:
            counter += lambda_

    best = toolbox.selectBest(population, 1)

    last_checkpoint = {
        'population': population,
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

    return population, best, history, local_history


def var_or(population, toolbox, lambda_, cross_prob, mut_prob):
    assert (cross_prob + mut_prob) <= 1.0, (
        'The sum of the crossover and mutation probabilities must be smaller '
        'or equal to 1.0.')

    offspring = []
    for _ in range(lambda_):
        op_choice = random.random()
        if op_choice < cross_prob:
            # Apply crossover
            ind1, ind2 = map(toolbox.clone, random.sample(population, 2))
            ind1, ind2 = toolbox.mate(ind1, ind2)

            del ind1.fitness.values
            offspring.append(ind1)

        elif op_choice < cross_prob + mut_prob:
            # Apply mutation
            ind = toolbox.clone(random.choice(population))
            ind, = toolbox.mutate(ind)

            del ind.fitness.values
            offspring.append(ind)

        else:
            # Apply reproduction
            offspring.append(random.choice(population))

    return offspring
