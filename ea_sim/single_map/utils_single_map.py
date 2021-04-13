import json
import numpy as np
import pickle
import random


def load_checkpoint(filename):
    # load evolution data from checkpoint
    with open(filename, 'rb') as cpf:
        cp_data = pickle.load(cpf)

        alg_data = {
            'last_num_sims': cp_data['num_sims'],
            'skips': cp_data['skips'],
            'num_gen': cp_data['num_gen'],
            'archive': cp_data['archive'],
            'dr_model': cp_data['dr_model']
        }
        last_nn_id, last_node_id = cp_data['last_nn_ids']
        entity_id_map = cp_data['entity_id_map']
        hof = cp_data['hall_of_fame']
        random.setstate(cp_data['rnd_state'])

    return alg_data, last_nn_id, last_node_id, entity_id_map, hof


def record_population(num_sims, population, entity_id_map, file, skips, pbar=None, verbose=False):
    individuals, fitness_values, _ = [list(t) for t in zip(*population)]

    pop_stats = {
        'num_sims': num_sims,
        'avg_fitness': np.mean(fitness_values),
        'std_dev': np.std(fitness_values),
        'min': np.min(fitness_values),
        'max': np.max(fitness_values)
    }

    # store the current population values
    file.write(json.dumps(pop_stats))

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
