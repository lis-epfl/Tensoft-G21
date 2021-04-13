import argparse
import copy
import neat
import numpy as np
import os
import graphviz
import warnings
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from goal_reaching.utils_gr import parse_controller


def draw_net(config, genome, view=False, filename=None, node_names=None, show_disabled=True, prune_unused=False,
             node_colors=None, fmt='png', avg=False):
    """ Receives a genome and draws a neural network with arbitrary topology. """
    # Attributes for network nodes.
    if graphviz is None:
        warnings.warn("This display is not available due to a missing optional dependency (graphviz)")
        return

    if node_names is None:
        node_names = {}

    assert type(node_names) is dict

    if node_colors is None:
        node_colors = {}

    assert type(node_colors) is dict

    node_attrs = {
        'shape': 'circle',
        'fontsize': '9',
        'height': '0.2',
        'width': '0.2'}

    dot = graphviz.Digraph(format=fmt, node_attr=node_attrs)

    inputs = set()
    for k in config.genome_config.input_keys:
        inputs.add(k)
        name = node_names.get(k, str(k))
        input_attrs = {'style': 'filled', 'shape': 'box', 'fillcolor': node_colors.get(k, 'lightgray')}
        dot.node(name, _attributes=input_attrs)


    outputs = set()
    for k in config.genome_config.output_keys:
        outputs.add(k)
        name = node_names.get(k, str(k))
        node_attrs = {'style': 'filled', 'fillcolor': node_colors.get(k, 'lightblue')}

        dot.node(name, _attributes=node_attrs)

    if prune_unused:
        connections = set()
        for cg in genome.connections.values():
            if cg.enabled or show_disabled:
                connections.add((cg.in_node_id, cg.out_node_id))

        used_nodes = copy.copy(outputs)
        pending = copy.copy(outputs)
        while pending:
            new_pending = set()
            for a, b in connections:
                if b in pending and a not in used_nodes:
                    new_pending.add(a)
                    used_nodes.add(a)
            pending = new_pending
    else:
        used_nodes = set(genome.nodes.keys())

    for n in used_nodes:
        if n in inputs or n in outputs:
            continue

        attrs = {'style': 'filled',
                 'fillcolor': node_colors.get(n, 'white')}
        dot.node(str(n), _attributes=attrs)

    for cg in genome.connections.values():
        if cg.enabled or show_disabled:
            # if cg.input not in used_nodes or cg.output not in used_nodes:
            #    continue
            input, output = cg.key
            a = node_names.get(input, str(input))
            b = node_names.get(output, str(output))
            style = 'solid' if cg.enabled else 'dotted'

            if avg:
                mean = float(np.mean(cg.weight))
                std = float(np.std(cg.weight))
                color = 'green' if mean > 0 else 'red'
                width = str(0.1 + abs(mean / 5.0))
                label = '{:.2f}\n STD: {:.2f}'.format(mean, std)
            else:
                color = 'green' if cg.weight > 0 else 'red'
                width = str(0.1 + abs(cg.weight / 5.0))
                label = '{:.2f}'.format(cg.weight)
            dot.edge(a, b, label=label, _attributes={'style': style, 'color': color,
                                                     'fontsize': '9', 'penwidth': width})

    dot.render(filename, view=view)

    return dot


def visualize_controller(controller_file, neat_settings_file):
    controller, node_names, num_inputs = parse_controller(controller_file, vis=True)

    neat_config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                              neat.DefaultSpeciesSet, neat.DefaultStagnation,
                              neat_settings_file)
    neat_config.genome_config.input_keys += [-(num_inputs + 1)]

    dot_index = controller_file.rfind('.')
    filename = controller_file[0: dot_index] if dot_index != -1 else controller_file

    draw_net(neat_config, controller, filename=filename, node_names=node_names)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for plotting a controller.')
    parser.add_argument('controller_file', metavar='controller_file', type=str, nargs='?',
                        default='controller_sim_0.txt',
                        help='the controller file')
    parser.add_argument('neat_settings_file', metavar='neat_settings_file', type=str, nargs='?',
                        default='neat_settings.txt',
                        help='neat settings file')
    args = parser.parse_args()

    visualize_controller(args.controller_file, args.neat_settings_file)
