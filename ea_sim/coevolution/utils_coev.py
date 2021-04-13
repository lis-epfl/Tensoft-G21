import pickle
import random

from algorithms import (vie,
                        mu_plus_lambda as mu_p_l,
                        map_elites)


def load_morph_checkpoint(filename, algorithm, num_sims):
    pop = None
    alg_data = None
    morphologies = {}
    hof = {}

    # load morphology evolution data from checkpoint
    with open(filename, 'rb') as cpf:
        cp_data = pickle.load(cpf)

        # extract data from the checkpoint according
        # to which algorithm generated it
        if algorithm['name'] == 'mu+lambda':
            pop = cp_data['population']
            morphologies = cp_data['morphologies']
            hof = cp_data['hall_of_fame']
            random.setstate(cp_data['rnd_state'])
            alg_data = {
                'last_num_sims': cp_data['num_sims'],
                'num_gen': cp_data['num_gen']
            }
        elif algorithm['name'] == 'vie':
            pop = cp_data['population']
            morphologies = cp_data['morphologies']
            hof = cp_data['hall_of_fame']
            random.setstate(cp_data['rnd_state'])
            alg_data = {
                'last_num_sims': cp_data['num_sims'],
                'num_gen': cp_data['num_gen'],
                'init_pop_size': cp_data['init_pop_size'],
                'current_boundary': cp_data['current_boundary'],
                'family_ids': cp_data['family_ids'],
                'prob': cp_data['prob']
            }
        elif algorithm['name'] == 'map-elites':
            morphologies = cp_data['morphologies']
            hof = cp_data['hall_of_fame']
            random.setstate(cp_data['rnd_state'])
            alg_data = {
                'last_num_sims': cp_data['num_sims'],
                'num_gen': cp_data['num_gen'],
                'archive': cp_data['archive'],
                'curiosity': cp_data['curiosity'],
            }
        else:
            raise Exception('Error! Unknown algorithm!')

    assert(num_sims == alg_data['last_num_sims'])

    return pop, alg_data, morphologies, hof


def run_coev(tbox, settings, checkpoint_dir, morph_evo_file, pop, alg_data,
             seed, morphologies, hof, min_num_modules):
    # execute specific EA depending on the configuration
    alg = settings.get('algorithm')
    TIME_NO_UPDATE = alg.get('time_no_update', 10000)

    if alg['name'] == 'mu+lambda':
        MU = alg.get('mu', 48)
        LAMBDA = alg.get('lambda', 48)
        CXPB = alg.get('cx_prob', 0.0)
        MUTPB = alg.get('mut_prob', 1.0)

        if pop is None:
            pop = tbox.population(MU)
        [pop, best, history, history_local] = \
            mu_p_l.run(pop, tbox, morph_evo_file, MU, LAMBDA,
                       CXPB, MUTPB, settings['max_num_sims'],
                       verbose=settings.get('verbose', True),
                       cp_freq=settings.get('checkpoint_freq', 10),
                       cp_folder=checkpoint_dir,
                       sim_id=0, seed=seed, alg_data=alg_data,
                       time_no_update=TIME_NO_UPDATE, coev=True,
                       eval_all=settings['eval_all'],
                       morphologies=morphologies,
                       hall_of_fame=hof)

    elif alg['name'] == 'vie':
        pop_size = alg.get('pop_size', 48)
        num_mutants = alg.get('num_mutants', 48)
        conv_rate = alg.get('conv_rate', 0.1)

        if pop is None:
            pop = tbox.population(pop_size)
        [pop, best, history, history_local] = \
            vie.run(pop, tbox, morph_evo_file, settings['max_num_sims'],
                    conv_rate, num_mutants,
                    verbose=settings.get('verbose', True),
                    cp_freq=settings.get('checkpoint_freq', 10),
                    cp_folder=checkpoint_dir,
                    sim_id=0, seed=seed, alg_data=alg_data, boundary_dist=1e-10,
                    time_no_update=TIME_NO_UPDATE, coev=True,
                    eval_all=settings['eval_all'],
                    morphologies=morphologies,
                    hall_of_fame=hof)

    elif alg['name'] == 'map-elites':
        num_init_sols = alg.get('num_init_sols', 1200)
        batch_size = alg.get('batch_size', 24)

        if pop is None:
            init_sols = tbox.population(num_init_sols)
        else:
            init_sols = [None for _ in range(num_init_sols)]

        # manage the 2D vs 3D case
        if settings['robot'].get('per_module_stiffness', False):
            map_dims = (settings['robot']['max_num_modules'] - min_num_modules + 1,
                        len(settings['robot']['modules_conf']['stiff_table']),
                        len(settings['robot']['modules_conf']['stiff_table']))
        else:
            map_dims = (settings['robot']['max_num_modules'] - min_num_modules + 1,
                        len(settings['robot']['modules_conf']['stiff_table']))

        [pop, history] = map_elites.run(init_sols, tbox, morph_evo_file, settings['max_num_sims'],
                                        map_dims, batch_size,
                                        verbose=settings.get('verbose', True),
                                        cp_folder=checkpoint_dir,
                                        cp_freq=settings.get('checkpoint_freq', 5),
                                        arch_folder=settings['result_dir'],
                                        use_curiosity=settings.get('use_curiosity', False),
                                        seed=seed, sim_id=0, alg_data=alg_data,
                                        time_no_update=TIME_NO_UPDATE, coev=True,
                                        morphologies=morphologies,
                                        hall_of_fame=hof)

    else:
        raise Exception('{} algorithm is not implemented!'.format(settings.get('algorithm')))


def get_last_c_id(contr_evo):
    c_ids = set()

    for gen in contr_evo:
        for ind in gen['population']:
            c_ids.add(ind['c_id'])

    return max(c_ids)


def update_history_and_hof(history_file, hall_of_fame, hof_max_size, generation, m_c_map,
                           morphologies_to_eval, genomes, fitness_values):
    index = 0
    for morph_indx in m_c_map.keys():
        for contr_indx in m_c_map[morph_indx]:
            m_id = morphologies_to_eval[morph_indx][0]
            c_id = genomes[contr_indx][0]
            fitness = fitness_values[index]
            distance = 1.0 / fitness

            history_file.write('{},{},{},{:.4f},{:.8f}\n'.format(generation, m_id, c_id, distance, fitness))
            update_hof(hall_of_fame, hof_max_size, m_id, c_id, distance)

            index += 1


def update_hof(hall_of_fame, max_size, m_id, c_id, distance):
    hof_size = 0
    for l in hall_of_fame.values():
        hof_size += len(l)

    if hof_size < max_size:
        if distance not in hall_of_fame:
            hall_of_fame[distance] = [(m_id, c_id)]
        elif (m_id, c_id) not in hall_of_fame[distance]:
            hall_of_fame[distance] += [(m_id, c_id)]
    elif distance < max(hall_of_fame.keys()):
        inserted = True
        if distance not in hall_of_fame:
            hall_of_fame[distance] = [(m_id, c_id)]
        elif (m_id, c_id) not in hall_of_fame[distance]:
            hall_of_fame[distance] += [(m_id, c_id)]
        else:
            inserted = False

        if inserted:
            max_dist = max(hall_of_fame.keys())

            del hall_of_fame[max_dist][-1]
            if len(hall_of_fame[max_dist]) == 0:
                del hall_of_fame[max_dist]
