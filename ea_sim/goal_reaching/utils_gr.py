import json
import numpy as np
import os

from neat.genome import DefaultGenome
from neat.genes import DefaultNodeGene, DefaultConnectionGene
from math import floor


def parse_controller(controller_filename, c_id=0, vis=False):
    names_list = ['T Distance', 'T Bearing', 'Obstacle', 'W Distance', 'W Bearing', 'Bias']

    controller = DefaultGenome(c_id)
    node_names = {}
    num_inputs = 0

    with open(controller_filename) as controller_file:
        lines = controller_file.readlines()
        for i in range(0, len(lines)):
            if i == 0:
                num_inputs = int(lines[i].split(' ')[0])
                for i in range(1, num_inputs+1):
                    node_names[-i] = names_list[i-1]
                if vis:
                    node_names[-(num_inputs+1)] = names_list[-1]
            elif i == 1:
                num_outputs = int(lines[i].split(' ')[0])
                for i in range(num_outputs):
                    node = DefaultNodeGene(i)
                    node.bias = 0.0
                    node.response = 1.0
                    node.aggregation = 'sum'
                    node.activation = 'sigmoid'
                    controller.nodes[i] = node

                    node_names[i] = 'm' + str(1 + floor(i / 2)) + '_' + ('f' if i % 2 == 0 else 'p')
            else:
                elements = lines[i].split(' ')
                if elements[0] != 'fitness:':
                    node_id = int(elements[0])
                    node_bias = float(elements[1])

                    if node_id in controller.nodes:
                        controller.nodes[node_id].bias = node_bias
                    else:
                        node = DefaultNodeGene(node_id)
                        node.bias = node_bias
                        node.response = 1.0
                        node.aggregation = 'sum'
                        node.activation = 'sigmoid'
                        controller.nodes[node_id] = node

                    if vis:
                        bias_conn_id = (-(num_inputs+1), node_id)
                        bias_connection = DefaultConnectionGene(bias_conn_id)
                        bias_connection.weight = node_bias
                        bias_connection.enabled = True
                        controller.connections[bias_conn_id] = bias_connection

                    num_connections = int(elements[2])
                    for i in range(num_connections):
                        conn_id = (int(elements[3 + 2 * i]), node_id)

                        connection = DefaultConnectionGene(conn_id)
                        connection.weight = float(elements[3 + 2 * i + 1])
                        connection.enabled = True

                        controller.connections[conn_id] = connection
                else:
                    break

    return controller, node_names, num_inputs


def open_history_file(filename, type=0):
    history = open(filename, 'w')

    if type == 0:
        history.write('gen_id,c_id,dist,inv_dist\n')
    elif type == 1:
        history.write('gen_id,m_id,c_id,dist,inv_dist\n')
    elif type == 2:
        history.write('gen_id,m_id,c_id,dist\n')

    return history


def restore_history_file(filename, last_checkpoint):
    history_up_to_checkpoint = ''

    sims_num = 0
    with open(filename, 'r') as history:
        for line in history:
            first_element = line.split(',')[0]

            if first_element == 'gen_id':
                history_up_to_checkpoint += line
            else:
                gen = int(first_element)
                if gen <= last_checkpoint:
                    sims_num += 1
                    history_up_to_checkpoint += line

    # restore history up to checkpoint
    with open(filename, 'w') as history:
        history.write(history_up_to_checkpoint)

    return open(filename, 'a'), sims_num


def open_evo_file(filename):
    evo_file = open(filename, 'w')
    # open the json list of generations
    evo_file.write('[')

    return evo_file


def restore_evo_file(filename, last_checkpoint, num_sims):
    success = True
    with open(filename, 'r') as f:
        try:
            evo_info = json.load(f)
        except json.JSONDecodeError:
            success = False

    if not success:
        with open(filename, 'a') as f:
            f.write(']')

        with open(filename, 'r') as f:
            evo_info = json.load(f)

    assert (num_sims == evo_info[last_checkpoint]['num_sims'])
    evo_info = evo_info[:last_checkpoint + 1]

    with open(filename, 'w') as f:
        f.write(json.dumps(evo_info))

    with open(filename, 'rb+') as f:
        f.seek(-1, os.SEEK_END)
        f.truncate()

    return open(filename, 'a'), evo_info


def restore_best_controller(contr_evo, controllers_dir, seed):
    fit_id_map = {}

    for gen in contr_evo:
        for ind in gen['population']:
            if ind['fitness'] is not None:
                if ind['fitness'] not in fit_id_map:
                    fit_id_map[ind['fitness']] = [ind['c_id']]
                elif ind['c_id'] not in fit_id_map[ind['fitness']]:
                    fit_id_map[ind['fitness']] += [ind['c_id']]

    best_fit = min(fit_id_map.keys())
    best_id = fit_id_map[best_fit][0]

    best_ind, _, _ = parse_controller(
        os.path.join(controllers_dir, 'controller_{}_{}.txt'.format(seed, best_id)),
        best_id
    )
    best_ind.fitness = 1.0 / best_fit

    return best_ind


def update_history(history_file, p, genomes, controllers_indices):
    for indx in controllers_indices:
        genome_id = genomes[indx][0]
        fitness = genomes[indx][1].fitness
        distance = 1.0 / fitness

        history_file.write(str(p.generation) + ',' + str(genome_id) + ',' + str(distance) + ',' + str(fitness) + '\n')


# logs information and statistics related to the current generation
def log_gen_info(contr_evo_file, num_sims, genomes, append):
    population = []
    distances = []

    for c_id, genome in genomes:
        distance = 1.0 / genome.fitness if genome.fitness != 0 else None

        population.append({
            "c_id": c_id,
            "fitness": distance
        })

        if distance is not None:
            distances.append(distance)

    gen_stats = {
        'num_sims': num_sims,
        'avg_fitness': np.mean(distances) if distances != [] else None,
        'std_dev': np.std(distances) if distances != [] else None,
        'min': np.min(distances) if distances != [] else None,
        'max': np.max(distances) if distances != [] else None,
        'population': population
    }

    if append:
        contr_evo_file.write(', ')
    contr_evo_file.write(json.dumps(gen_stats))



