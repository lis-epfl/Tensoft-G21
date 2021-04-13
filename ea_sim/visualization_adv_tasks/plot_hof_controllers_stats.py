import argparse
import copy
import json
import math
import neat
import os
import sys

from neat.genome import DefaultGenome
from neat.genes import DefaultNodeGene, DefaultConnectionGene

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from goal_reaching.utils_gr import parse_controller
from utils import parse_robot_string
from visualization_adv_tasks import visualize_controller
from visualization.plot_actuation_stats import plot_stats_boxplot


def plot_hof_controllers_stats(results_dir, seed, neat_settings, hall_of_fame_file, morphologies_file):
    # create results directory
    hall_of_fame_stats_dir = os.path.join(results_dir, 'hall_of_fame_stats')
    contr_stats_dir = os.path.join(hall_of_fame_stats_dir, 'controllers')
    os.makedirs(hall_of_fame_stats_dir, exist_ok=True)
    os.makedirs(contr_stats_dir, exist_ok=True)

    with open(hall_of_fame_file) as hof_file:
        hall_of_fame = json.load(hof_file)

    with open(morphologies_file) as morph_file:
        morphologies = json.load(morph_file)
    inverse_morph_dict = {v: k for k, v in morphologies.items()}

    neat_config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                              neat.DefaultSpeciesSet, neat.DefaultStagnation,
                              neat_settings)

    controllers = []
    for pair_list in hall_of_fame.values():
        for m_id, c_id in pair_list:
            genome, _, _ = parse_controller(
                os.path.join(results_dir, 'controllers', 'controller_{}_{}.txt'.format(seed, c_id)),
                c_id
            )
            controller_nn = neat.nn.FeedForwardNetwork.create(genome, neat_config)
            morphology = parse_robot_string(inverse_morph_dict[m_id])
            controllers.append((genome, controller_nn, len(morphology)))

    avg_genome, avg_neat_config, node_names = init_avg_genome(neat_config)

    i_o_rates, i_h_rates, h_h_rates, h_o_rates, o_h_rates, o_o_rates = \
        compute_rates(controllers, avg_genome=avg_genome, avg_neat_config=avg_neat_config)

    conf = {
        'data': [i_o_rates, i_h_rates, h_h_rates, h_o_rates, o_h_rates, o_o_rates],
        'title': 'Connections usage across Hall of Fame',
        'x_tick_labels': [
            'Input-Output', 'Input-Hidden', 'Hidden-Hidden', 'Hidden-Output', 'Output-Hidden', 'Output-Output'
        ],
        'y_label': 'Conn. usage ratio',
        'ratio': True,
        'out_file': os.path.join(contr_stats_dir, 'conn_usage_{}.pdf'.format(seed))
    }
    plot_stats_boxplot(conf)

    visualize_controller.draw_net(avg_neat_config, avg_genome,
                                  filename=os.path.join(contr_stats_dir, 'avg_controller_{}'.format(seed)),
                                  node_names=node_names, avg=True)


def init_avg_genome(neat_config):
    names_list = ['T Distance', 'T Bearing', 'Obstacle', 'W Distance', 'W Bearing', 'Bias']

    avg_genome = DefaultGenome(-1)

    avg_neat_config = copy.deepcopy(neat_config)
    avg_neat_config.genome_config.num_inputs = neat_config.genome_config.num_inputs + 1
    avg_neat_config.genome_config.input_keys += [-(neat_config.genome_config.num_inputs + 1)]

    node_names = {}
    for i in range(1, avg_neat_config.genome_config.num_inputs):
        node_names[-i] = names_list[i - 1]
    node_names[-avg_neat_config.genome_config.num_inputs] = names_list[-1]

    for i in range(avg_neat_config.genome_config.num_outputs):
        node_names[i] = 'm' + str(1 + math.floor(i / 2)) + '_' + ('f' if i % 2 == 0 else 'p')

    return avg_genome, avg_neat_config, node_names


def compute_rates(controllers, avg_genome=None, avg_neat_config=None):
    i_o_rates = []
    i_h_rates = []
    h_h_rates = []
    h_o_rates = []
    o_h_rates = []
    o_o_rates = []

    for genome, nn, num_modules in controllers:
        i_nodes = nn.input_nodes
        o_nodes = nn.output_nodes
        h_nodes = [node for node in genome.nodes.keys() if node not in i_nodes and node not in o_nodes]

        red_h_nodes, red_o_nodes = reduce_controller(genome, i_nodes, h_nodes, o_nodes, num_modules)

        if avg_genome is not None and avg_neat_config is not None:
            merge_genomes(avg_genome, genome, avg_neat_config.genome_config.input_keys[-1])

        i_o_conn = 0
        i_h_conn = 0
        h_h_conn = 0
        h_o_conn = 0
        o_h_conn = 0
        o_o_conn = 0

        for n1, n2 in genome.connections.keys():
            if n1 in i_nodes and n2 in red_o_nodes:
                i_o_conn += 1
            elif n1 in i_nodes and n2 in red_h_nodes:
                i_h_conn += 1
            elif n1 in red_h_nodes and n2 in red_h_nodes:
                h_h_conn += 1
            elif n1 in red_h_nodes and n2 in red_o_nodes:
                h_o_conn += 1
            elif n1 in red_o_nodes and n2 in red_h_nodes:
                o_h_conn += 1
            elif n1 in red_o_nodes and n2 in red_o_nodes:
                o_o_conn += 1
            else:
                print('Error in pair ({}, {}) for genome {}'.format(n1, n2, genome.key))
                exit(0)

        i_o_rates.append(i_o_conn / (len(i_nodes) * len(red_o_nodes)))

        i_h_rate = (i_h_conn / (len(i_nodes) * len(red_h_nodes))) if len(red_h_nodes) > 0 else 0
        i_h_rates.append(i_h_rate)

        h_h_max = ((len(red_h_nodes)-1) * len(red_h_nodes)) / 2
        h_h_rate = (h_h_conn / h_h_max) if h_h_max > 0 else 0
        h_h_rates.append(h_h_rate)

        h_o_rate = (h_o_conn / (len(red_h_nodes) * len(red_o_nodes))) if len(red_h_nodes) > 0 else 0
        h_o_rates.append(h_o_rate)

        o_h_max = (len(red_o_nodes)-1) * len(red_h_nodes)
        o_h_rate = (o_h_conn / o_h_max) if o_h_max > 0 else 0
        o_h_rates.append(o_h_rate)

        o_o_rate = (o_o_conn / (((len(red_o_nodes) - 1) * len(red_o_nodes)) / 2)) if len(red_o_nodes) > 1 else 0
        o_o_rates.append(o_o_rate)

    return i_o_rates, i_h_rates, h_h_rates, h_o_rates, o_h_rates, o_o_rates


def merge_genomes(avg_genome, genome, bias_id):
    # merge nodes
    for node_id, node_gene in genome.nodes.items():
        if node_id not in avg_genome.nodes:
            node = DefaultNodeGene(node_id)
            node.bias = 0.0
            node.response = 1.0
            node.aggregation = 'sum'
            node.activation = 'sigmoid'
            avg_genome.nodes[node_id] = node

        if (bias_id, node_id) in avg_genome.connections:
            avg_genome.connections[(bias_id, node_id)].weight.append(node_gene.bias)
        else:
            connection = DefaultConnectionGene((bias_id, node_id))
            connection.weight = [node_gene.bias]
            connection.enabled = True
            avg_genome.connections[(bias_id, node_id)] = connection

    # merge edges
    for conn_id, conn_gene in genome.connections.items():
        if conn_id in avg_genome.connections:
            avg_genome.connections[conn_id].weight.append(conn_gene.weight)
        else:
            connection = DefaultConnectionGene(conn_id)
            connection.weight = [conn_gene.weight]
            connection.enabled = True
            avg_genome.connections[conn_id] = connection


def reduce_controller(genome, i_nodes, h_nodes, o_nodes, num_modules):
    hidden_nodes = copy.deepcopy(h_nodes)
    output_nodes = copy.deepcopy(o_nodes)

    from_dict = {}
    to_dict = {}

    for n in i_nodes+hidden_nodes+output_nodes:
        from_dict[n] = []
        to_dict[n] = []

    for n1, n2 in genome.connections.keys():
        from_dict[n1].append(n2)
        to_dict[n2].append(n1)

    for to_node in range(2*num_modules, len(output_nodes)):
        if to_node in output_nodes and len(from_dict[to_node]) == 0:
            remove_connections(genome, to_node, from_dict, to_dict, hidden_nodes, output_nodes, num_modules)
            output_nodes.remove(to_node)
            del genome.nodes[to_node]

    hidden_nodes_to_test = copy.deepcopy(hidden_nodes)
    for i in range(0, len(hidden_nodes_to_test)):
        to_node = hidden_nodes_to_test[i]
        if to_node in hidden_nodes and len(from_dict[to_node]) == 0:
            remove_connections(genome, to_node, from_dict, to_dict, hidden_nodes, output_nodes, num_modules)
            hidden_nodes.remove(to_node)
            del genome.nodes[to_node]

    return hidden_nodes, output_nodes


def remove_connections(genome, to_node, from_dict, to_dict, hidden_nodes, output_nodes, num_modules):
    for j in range(len(to_dict[to_node])-1, -1, -1):
        from_node = to_dict[to_node][j]

        del genome.connections[(from_node, to_node)]

        from_dict[from_node].remove(to_node)
        to_dict[to_node].remove(from_node)

        if from_node in hidden_nodes and len(from_dict[from_node]) == 0:
            remove_connections(genome, from_node, from_dict, to_dict, hidden_nodes, output_nodes, num_modules)
            hidden_nodes.remove(from_node)
            del genome.nodes[from_node]
        elif (from_node in output_nodes and from_node >= 2*num_modules) and len(from_dict[from_node]) == 0:
            remove_connections(genome, from_node, from_dict, to_dict, hidden_nodes, output_nodes, num_modules)
            output_nodes.remove(from_node)
            del genome.nodes[from_node]


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for generating plots of controllers stats about the '
                                                 'individuals contained in the hall of fame '
                                                 '(co-evolution & double map).')
    parser.add_argument('res_dir', metavar='res_dir', type=str, nargs='?',
                        default='res_dir', help='the folder where results are stored')
    args = parser.parse_args()

    settings_file = os.path.join(args.res_dir, 'settings.json')
    with open(settings_file) as sf:
        settings = json.load(sf)
    neat_settings = os.path.join(args.res_dir, 'neat_settings.txt')

    for seed in settings['seeds']:
        hall_of_fame_file = os.path.join(args.res_dir, 'best', 'hall_of_fame_{}.json'.format(seed))
        morphologies_file = os.path.join(args.res_dir, 'morphologies', 'morphologies_{}.json'.format(seed))

        plot_hof_controllers_stats(args.res_dir, seed, neat_settings, hall_of_fame_file, morphologies_file)
