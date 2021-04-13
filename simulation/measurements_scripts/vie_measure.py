import os
import random
import numpy as np

# ======================================= #
#           Auxiliary  Functions          #
# ======================================= #
def check_families(population, family_ids):
    # transform to set and then back to list to avoid duplicates
    return list(set([ind.ID for ind in population if ind.ID in family_ids]))


def get_prob_ids(population, list_ids):
    """

    :param population:
    :param list_ids:
    :return:
    """
    prob = {k:0 for k in list_ids}
    prob_sum = 0

    for ind in population:
        prob[ind.ID] += 1

    for l_id in list_ids:
        prob[l_id] = 1 / float(prob[l_id])
        prob_sum += prob[l_id]

    for l_id in list_ids:
        prob[l_id] /= prob_sum

    return prob


def generate_mutant(population, toolbox, family_IDs, prob):
    family_ID = np.random.choice(family_IDs, 1,
                                 p=[prob[f_id] for f_id in family_IDs])[0]
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
    # eliminate outside the boundaries
    indices = [i for i, ind in enumerate(population)
               if ind.fitness.values[0] > current_boundary or not ind.check_viability() ]

    # delete in a backward fashion to correctly update list indices
    for i in sorted(indices, reverse=True):
        del population[i]

    # check if a family died
    family_ids = check_families(population, family_ids)
    prob = get_prob_ids(population, family_ids)

    return family_ids, prob


def best_in_family(population, toolbox, best_id):
    """

    :param population:
    :param toolbox:
    :param best_id:
    :return:
    """
    family = []
    for ind in population:
        if ind.ID == best_id:
            family.append(ind)

    best = toolbox.selectBest(family, 1)

    return best[0]


def evaluate_ind(toolbox, individuals):
    """
    :param toolbox:
    :param individuals:
    :return:
    """
    # consider only new individuals (offsprings)
    invalid_ind = [ind for ind in individuals if not ind.fitness.valid]

    if len(invalid_ind) > 0:
        fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

    return invalid_ind


def print_stats(num_sims, population):
    fitness_values = np.asarray(list(map(lambda i: i.fitness.values, population)))

    pop_stats = {
        'num_sims': num_sims,
        'avg_fitness': np.mean(fitness_values),
        'std_dev': np.std(fitness_values),
        'min': np.min(fitness_values),
        'max': np.max(fitness_values)
    }

    print('num_sims: {} | Fitness -> avg: {} std: {} min: {} max: {}'.format(
        pop_stats['num_sims'], pop_stats['avg_fitness'],
        pop_stats['std_dev'], pop_stats['min'], pop_stats['max']
    ))

    # if population variability decrease under a certain threshold, stop the evolution
    return pop_stats['std_dev'] < 0.001


# ======================================= #
#           Viability Algorithm           #
# ======================================= #
def run(population, toolbox, max_num_sims, convergence_rate=0.1,
        num_mutants=24, min_update=0.1):
    """

    :param population:
    :param toolbox:
    :param convergence_rate:
    :param max_num_sims:
    :param num_mutants: represents the number of individuals
                        that are reproduced within the inner loop
                        (a good value could be half the population)
    :param min_update: the minumum difference between current best
                        and the boundary to continue the evolution
    :return:
    """
    num_sims = 0

    for ID, ind in enumerate(population):
        ind.ID = ID

    # Evaluate the initial random population
    evaluate_ind(toolbox, population)
    num_sims += len(population)

    # Store all information of individuals of initial generation
    print_stats(num_sims, population)

    # initialize the details regarding boundaries and families (minimization version)
    init_boundary = (max(population, key=lambda x: x.fitness.values)).fitness.values[0]
    current_boundary = init_boundary

    init_pop_size = len(population)
    family_ids = list(range(init_pop_size))
    prob = get_prob_ids(population, family_ids)
    best_fam_inds = []

    converged = False

    while num_sims < max_num_sims and not converged:
        while len(population) < 2 * init_pop_size and not converged:
            mutants = []

            # create mutants
            for _ in range(num_mutants):
                mutants.append(generate_mutant(population, toolbox,
                                               family_ids, prob))

            # calculate fitness and update population
            population += evaluate_ind(toolbox, mutants)
            num_sims += num_mutants

            # note: family_ids is recreated within the function
            family_ids, prob = eliminate_non_viable(population,
                                                    current_boundary,
                                                    family_ids)

            # check whether the algorithm has finished its budget
            best_fit = (min(population, key=lambda x: x.fitness.values)).fitness.values[0]
            if num_sims >= max_num_sims or np.abs(best_fit - current_boundary) < min_update:
                converged = True

            if num_sims % init_pop_size == 0:
                converged = converged or print_stats(num_sims, population)

                if num_sims % init_pop_size*10 == 0:
                    with open(os.path.join('outputs', 'module_evo_results_tmp.csv'), 'w') as out_file:
                        out_file.write('fitness,K1,K4,K6,K7,K10\n')
                        for ind in sorted(population):
                            out_file.write(str(ind.fitness.values[0]) + ',' + ','.join(str(ind).split(' ')) + '\n')

        # update boundaries
        best_fit = (min(population, key=lambda x: x.fitness.values)).fitness.values[0]
        current_boundary -= float(np.abs(best_fit - current_boundary)) * convergence_rate

        family_ids, prob = eliminate_non_viable(population,
                                                current_boundary,
                                                family_ids)

        best_fam_inds = [best_in_family(population, toolbox, familyID)
                         for familyID in family_ids]

    return population, best_fam_inds
