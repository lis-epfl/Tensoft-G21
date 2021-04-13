import os
import random

from tqdm import tqdm
from utils import (store_checkpoint, record_population,
                   evaluate_ind, load_history, store_history)


# ======================================= #
#           Auxiliary  Functions          #
# ======================================= #
def check_families(population, family_ids):
    """ Filter from current set of families the ones whose lineage has extinguished.

    :param population: list of evolved individuals
    :param family_ids: list of unique family ids
    :return: the new list of family ids
    """
    return list(set([ind.ID for ind in population if ind.ID in family_ids]))


def get_family_prob(population, family_ids):
    """ Compute for each family the probabily of being chosen
        for breeding new individuals.

    :param population: list of evolved individuals
    :param family_ids: list of unique family ids
    :return: a dictionary mapping family ids to their selection probability
    """
    prob = { k: 0 for k in family_ids }
    prob_sum = 0

    for ind in population:
        prob[ind.ID] += 1

    for l_id in family_ids:
        prob[l_id] = 1 / float(prob[l_id])
        prob_sum += prob[l_id]

    for l_id in family_ids:
        prob[l_id] /= prob_sum

    return prob


def generate_mutant(population, toolbox, family_ids, prob):
    """ Randomly select from one family one parent,
        which is employed for breeding a new individual.

    :param population: list of evolved individuals
    :param toolbox: DEAP instance for using EA operators
    :param family_ids: list of unique family ids
    :param prob: a map family ids - selection probability
    :return: a new population individual
    """
    family_ID = random.choices(family_ids, [prob[f_id] for f_id in family_ids], k=1)[0]
    family = [ind for ind in population if ind.ID == family_ID]
    parent = random.choice(family)

    child = toolbox.clone(parent)
    # try to mutate the robot until one mutation actually occurs
    while child == parent:
        child, = toolbox.mutate(child)

    # delete fitness value of the offspring inherited from its parent,
    # so that the child can be evaluated
    del child.fitness.values
    return child


def eliminate_non_viable(population, current_boundary, family_ids):
    """ Remove from the population all the individuals that do not meet
        the viability conditions.

    :param population: list of evolved individuals
    :param current_boundary: the set of values that defines the viability of a solution
    :param family_ids: list of unique family ids
    :return: updated list of family ids and corresponding probabilities
    """
    # eliminate individuals outside the boundaries (max. problem)
    indices = [i for i, ind in enumerate(population)
               if ind.fitness.values[0] < current_boundary]

    # delete in a backward fashion to correctly update list indices
    for i in sorted(indices, reverse=True):
        del population[i]

    # check if a family died
    family_ids = check_families(population, family_ids)
    prob = get_family_prob(population, family_ids)

    return family_ids, prob


def best_in_family(population, toolbox, best_id):
    """ Select from given family the fittest individual

    :param population: list of evolved individuals
    :param toolbox: DEAP instance for using EA operators
    :param best_id: id of the family from which individuals are drawn
    :return: the individual of selected family with highest fitness
    """
    family = []
    for ind in population:
        if ind.ID == best_id:
            family.append(ind)

    best = toolbox.selectBest(family, 1)

    return best[0]


# ======================================= #
#           Viability Algorithm           #
# ======================================= #
def run(population, toolbox, evo_file, max_num_sims, convergence_rate=0.1,
        num_mutants=24, verbose=__debug__, cp_freq=20, cp_folder='',
        history_file=None, sim_id=None, seed=0, alg_data=None, boundary_dist=0.1,
        time_no_update=10000, coev=False, eval_all=False, morphologies=None,
        hall_of_fame=None):
    """ Execute the Viability Evolution algorithm.

    :param population: list of evolved individuals
    :param toolbox: DEAP instance for using EA operators
    :param evo_file: the file descriptor where are stored all the collected
                     information about the evolutionary process
    :param convergence_rate: the percentage of the difference between best fitness
                             and current boundary that is adopted to update the boundary
    :param max_num_sims: maximum number of different solutions that can be tested.
                         It defines the termination criteria of the algorithm
    :param num_mutants: represents the number of individuals
                        that are reproduced at each time
                        (a good value could be half the population).
                        This must be within 1 and the population size.
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
    :param boundary_dist: minimum distance between current best and boundary (if the distance is lower
                    the simulation will be stopped)
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
        num_sims = alg_data['last_num_sims']
        num_gen = alg_data['num_gen']
        current_boundary = alg_data['current_boundary']
        init_pop_size = alg_data['init_pop_size']
        family_ids = alg_data['family_ids']
        prob = alg_data['prob']

        pbar.update(num_sims)
        skips = num_sims
        print('Restart evolution from {}'.format(num_sims))
        print('Population size: {}'.format(len(population)))
        print('Current boundary: {}'.format(current_boundary))
        print('Family IDs: {}'.format(family_ids))
        print('Family IDs Probabilities: {}'.format(prob))
    else:
        num_sims = 0
        if num_mutants < 1 or num_mutants > len(population):
            raise Exception(f'Current batch size for generating new individuals ({num_mutants})'
                            'is either larger than initial population size or empty.')

        # assign a different family identifier to each population's individual
        for ID, ind in enumerate(population):
            ind.ID = ID

        # Evaluate the initial random population
        _, skips, n_eval = evaluate_ind(toolbox, population, history, local_history,
                                        coev=coev, eval_all=eval_all)

        pbar.update(min(n_eval, max_num_sims - num_sims))
        num_sims += n_eval
        num_gen = 0

        # Store all information of individuals of initial generation
        record_population(num_sims, population, evo_file, skips,
                          pbar=pbar, verbose=verbose, coev=coev)

        # initialize the details regarding boundaries and families
        init_boundary = (min(population, key=lambda x: x.fitness.values)).fitness.values[0]
        current_boundary = init_boundary

        init_pop_size = len(population)
        family_ids = list(range(init_pop_size))
        prob = get_family_prob(population, family_ids)

    best_fam_inds = []
    last_best_update = num_sims
    best_fit = (max(population, key=lambda x: x.fitness.values)).fitness.values[0]
    converged = False
    gen_rate = init_pop_size / num_mutants

    while num_sims < max_num_sims and not converged:
        while len(population) < 2 * init_pop_size and not converged:
            mutants = []

            # create mutants
            for _ in range(num_mutants):
                mutants.append(generate_mutant(population, toolbox, family_ids, prob))

            # calculate fitness and update population
            if coev and eval_all:
                evaluated, cskips, n_eval = evaluate_ind(toolbox, population + mutants, history,
                                                         local_history, coev=coev, eval_all=eval_all)
                population = evaluated
            else:
                evaluated, cskips, n_eval = evaluate_ind(toolbox, mutants, history, local_history,
                                                         coev=coev, eval_all=eval_all)
                population += evaluated
            skips += cskips
            pbar.update(min(n_eval, max_num_sims - num_sims))
            num_sims += n_eval
            num_gen += 1

            # note: family_ids is recreated within the function
            family_ids, prob = eliminate_non_viable(population,
                                                    current_boundary, family_ids)

            # check whether the algorithm has finished its budget
            if num_sims >= max_num_sims:
                converged = True

            # after a generation has been processed, log its details
            if not coev and (num_sims % init_pop_size == 0) or coev:
                # write a comma to separate generation data in json file
                evo_file.write(', ')
                # store all information of individuals of current generation
                record_population(num_sims, population, evo_file, skips,
                                  pbar=pbar, verbose=verbose, coev=coev)

                # store checkpoint
                if not coev and (num_sims % (cp_freq*init_pop_size) == 0) or \
                        coev and ((num_gen+gen_rate) % (gen_rate*cp_freq) == 0):
                    store_checkpoint(
                        {
                            'population': population,
                            'num_sims': num_sims,
                            'num_gen': num_gen,
                            'rnd_state': random.getstate(),
                            'init_pop_size': init_pop_size,
                            'current_boundary': current_boundary,
                            'family_ids': family_ids,
                            'prob': prob,
                            'morphologies': morphologies,
                            'hall_of_fame': hall_of_fame
                        },
                        os.path.join(cp_folder, 'checkpoint_{}_{}.pkl'.format(seed, sim_id))
                    )
                    # save a temporary history that can be used
                    # in case of execution interruptions
                    if not coev:
                        store_history(history, os.path.join(cp_folder, 'history_tmp.csv'))

        # update boundaries
        curr_best_fit = (max(population, key=lambda x: x.fitness.values)).fitness.values[0]
        current_boundary += float(curr_best_fit - current_boundary) * convergence_rate

        # termination criteria based on fitness aging
        curr_best_fit = (max(population, key=lambda x: x.fitness.values)).fitness.values[0]
        if curr_best_fit > best_fit:
            best_fit = curr_best_fit
            last_best_update = num_sims

        if current_boundary >= (curr_best_fit - boundary_dist) or (num_sims - last_best_update) >= time_no_update:
            converged = True

        family_ids, prob = eliminate_non_viable(population,
                                                current_boundary, family_ids)

        best_fam_inds = [best_in_family(population, toolbox, familyID)
                         for familyID in family_ids]

    last_checkpoint = {
        'population': population,
        'num_sims': num_sims,
        'num_gen': num_gen,
        'rnd_state': random.getstate(),
        'init_pop_size': init_pop_size,
        'current_boundary': current_boundary,
        'family_ids': family_ids,
        'prob': prob,
        'morphologies': morphologies,
        'hall_of_fame': hall_of_fame
    }

    # save latest data that may have been updated by latest elimination
    store_checkpoint(
        last_checkpoint,
        os.path.join(cp_folder, 'checkpoint_{}_{}.pkl'.format(seed, sim_id))
    )

    pbar.close()

    return population, best_fam_inds, history, local_history
