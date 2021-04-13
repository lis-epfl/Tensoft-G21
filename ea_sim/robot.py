import os
import random
import subprocess
import numpy as np
import pygraphviz as pgv

from functools import reduce
from math import isclose
from params_conf import MIN_NUM_MODULES

class Module:
    def __init__(self, order, mod_type, stiffness, stiff_idx=None):
        self.order = order
        self.type = mod_type
        # NOTE: rotation parameter no more used but kept
        # to avoid changing simulation input function TODO: update the code removing it
        self.rotation = 0
        # use the same robot stiffness for all the modules
        self.stiff_idx = stiff_idx
        self.stiffness = stiffness
        self.child_module = 0
        # the module face where the child is attached
        self.attached_face = 0

    def __eq__(self, other):
        """ Check whether this module is equivalent
            (has the same values) to the other.

        :param other:
        :return:
        """
        return isinstance(other, self.__class__) and self.order == other.order \
            and self.order == other.order and self.type == other.type \
            and self.rotation == other.rotation \
            and self.stiff_idx == other.stiff_idx \
            and isclose(self.stiffness, other.stiffness, rel_tol=1e-05) \
            and self.order == other.order and self.type == other.type \
            and self.child_module == other.child_module \
            and self.attached_face == other.attached_face

    def get_hr_repr(self, type_table):
        """ Get a human readable representation of this module

        :param type_table:
        :return:
        """
        output = 'order: {} connectedModules: {} connectedFaces: {} '.format(
            self.order,
            self.child_module,
            self.attached_face
        )

        output += 'freq: {} amplitude: {} phase: {} rot: {} stiff: {}\n'.format(
            type_table[self.type]['freq'],
            type_table[self.type]['amp'],
            type_table[self.type]['phase'],
            self.rotation,
            self.stiffness
        )

        return output

    def get_info_repr(self, type_table):
        """ Get a representation of this module

        :param type_table:
        :return:
        """
        return {
            'order': self.order,
            'freq': type_table[self.type]['freq'],
            'amplitude': type_table[self.type]['amp'],
            'phase': type_table[self.type]['phase'],
            'rot': self.rotation,
            'stiff': self.stiffness
        }

    def get_full_info(self, type_table):
        mod_repr = self.get_info_repr(type_table)

        mod_repr['connectedModules'] = self.child_module
        mod_repr['connectedFaces'] = self.attached_face

        return mod_repr


class Robot(list):
    def __init__(self, simulation_path, tracker_path, noise_type=0,
                 noise_level=1.0, num_faces=8, max_num_modules=10,
                 mutation_config=None, modules_conf=None, robot_tests=3,
                 per_module_stiffness=False, sim_seed=42):
        super().__init__()

        # simulation related parameters
        self.simulation_path = simulation_path
        self.tracker_path = tracker_path
        self.noise_type = noise_type
        self.noise_level = noise_level
        self.robot_tests = max(1, robot_tests)
        self.sim_seed = sim_seed

        self.num_faces = num_faces
        self.num_modules = random.randint(MIN_NUM_MODULES, max_num_modules)
        self.max_num_modules = max_num_modules

        self.modules = [None for _ in range(self.num_modules)]

        # probabilities levels for mutation function
        self._assign_mut_probs(mutation_config)

        if modules_conf is None:
            modules_conf = {}
        self._set_config(modules_conf)

        self.per_module_stiffness = per_module_stiffness
        self.stiff_idx = random.randint(0, self.max_stiffness)
        self.stiffness = self.stiff_table[self.stiff_idx]

        self._assign_parents()
        self._assign_faces()

    def _assign_mut_probs(self, mut_config):
        """ Compute the mutation probabilities for the robot

        :param mut_config:
        :return:
        """
        weights = {
            'global': [1, 1, 1],
            'local': [1, 1]
        }
        if mut_config is None:
            self.p_global_mut = 0.3
            self.p_local_mut = 0.6
            self.weights = weights
        else:
            self.p_global_mut = mut_config.get('p_global_mut', 0.3)
            self.p_local_mut = mut_config.get('p_local_mut', 0.6)
            self.weights = mut_config.get('weights', weights)

        if not 0 <= self.p_global_mut <= 1 or not 0 <= self.p_local_mut <= 1:
            raise Exception('Some of given mutation probability are not in the range [0-1]!')

        # bring the weights into the interval [0-1]
        self.weights['global'] = \
            np.asarray(self.weights['global']) / np.sum(self.weights['global'])
        self.weights['local'] = \
            np.asarray(self.weights['local']) / np.sum(self.weights['local'])

    def _set_config(self, config):
        """ Load which kind of configurations the robot can assume.

        :param config:
        :return:
        """
        range_freq = config.get('range_freq', [0.25, 0.5])
        range_amp = config.get('range_amp', [0.6])
        range_phase = config.get('range_phase', [0, 1.57, 3.14, 4.71])

        self.type_table = [{'freq': 0, 'amp': 0, 'phase': 0}]
        for f in range_freq:
            for a in range_amp:
                for p in range_phase:
                    self.type_table.append({'freq': f, 'amp': a, 'phase': p})

        self.max_types_table = len(self.type_table) - 1

        self.stiff_table = config['stiff_table']
        self.max_stiffness = len(self.stiff_table) - 1

    def _assign_parents(self):
        """ Generate the modules that compose the robot and connect them together. """
        for i in range(self.num_modules):
            # initialize a new module
            idx = None
            if self.per_module_stiffness:
                idx = random.randint(0, self.max_stiffness)
                stiff = self.stiff_table[idx]
            else:
                stiff = self.stiffness
            module = Module(i + 1, random.randint(0, self.max_types_table),
                            stiff, stiff_idx=idx)

            # assign child to all the module in the chain, except for the last one
            if module.order < self.num_modules:
                module.child_module = module.order + 1

            self.modules[i] = module

    def _assign_faces(self):
        """ Assign a random face on which module child is attached.
            Note: this value is also used for defining module rotation
        """
        for module in self.modules:
            module.attached_face = random.choice(range(1, self.num_faces + 1))

    def __eq__(self, other):
        """ Check whether this robot is equivalent
            (has the same parameters and structure) to the other.

        :param other:
        :return:
        """
        if not isinstance(other, self.__class__) \
                or self.num_modules != other.num_modules \
                or not isclose(self.stiffness, other.stiffness, abs_tol=1e-05):
            return False

        for a, b in zip(self.modules, other.modules):
            if a != b:
                return False

        return True

    def bd(self):
        return self.features_descriptor()

    def info(self, coev=False):
        """ Return robot information to be logged. """
        return {
            'fitness': self.fitness.values[0] if not coev else 1.0 / self.fitness.values[0],
            'num_modules': self.num_modules,
            'stiffness': self.stiffness,
            'modules': [m.get_info_repr(self.type_table) for m in self.modules],
            'sim_string': self.string_input()
        }

    def log_info(self):
        return {
            'num_modules': self.num_modules,
            'stiffness': self.stiffness,
            'modules': [m.get_info_repr(self.type_table) for m in self.modules],
            'sim_string': self.string_input()
        }

    def get_fitness(self):
        return self.fitness.values[0]

    def get_modules_conf(self):
        return [m.get_full_info(self.type_table) for m in self.modules]

    def features_descriptor(self):
        """ Return the feature descriptor depending whether
            it is a global vs multi-stiffness (2D vs 3D) set-up.
            In the first case a robot is identified by its number of modules
            that composed it and the stiffness level of all its modules.
            On the other hand, the latter take in consideration two stiffness values,
            the minimum and maximum among all the modules.
        """
        if self.per_module_stiffness:
            stiff_idxs =  [mod.stiff_idx for mod in self.modules]
            return self.num_modules - MIN_NUM_MODULES, min(stiff_idxs), max(stiff_idxs)
        else:
            return self.num_modules - MIN_NUM_MODULES, self.stiff_idx

    def features_desc_str(self):
        """ String version of features_descriptor function."""
        if self.per_module_stiffness:
            stiffs = [str(mod.stiffness) for mod in self.modules]
            return '{}_{}'.format(self.num_modules,
                                  '_'.join([min(stiffs), max(stiffs)]))
        else:
            return '{}_{}'.format(self.num_modules, self.stiffness)

    def switch_type(self, order):
        """ Switch randomly the type of selected module

        :param order:
        """
        self.modules[order - 1].type = random.randint(0, self.max_types_table)

    def switch_rotation(self, order):
        """ Switch randomly the rotation of a module (by changing which face is connected)

        :param order:
        """
        self.modules[order - 1].attached_face = random.choice(range(1, self.num_faces + 1))

    def add_module(self, stiffness, stiff_idx=None):
        """ Add a new random module in a random position
        (only if it does not exceed maximum allowed number of modules)
        :param stiffness:
        :param stiff_idx:
        """
        if len(self.modules) < self.max_num_modules:
            # select in which position add the new module
            idx = random.choice(range(self.num_modules))

            # create a new module to insert in that position
            module = Module(idx + 1,
                            random.randint(0, self.max_types_table),
                            stiffness, stiff_idx=stiff_idx)

            # update module description
            module.child_module = module.order + 1
            module.attached_face = random.choice(range(1, self.num_faces + 1))

            self.modules.insert(idx, module)

            # update modules IDs
            for i in range(idx + 1, len(self.modules)):
                self.modules[i].order = i + 1
                self.modules[i].child_module = self.modules[i].order + 1

            # reset the child of last module, which is not attached to anything
            # on the contrary, the attached face is kept to preserve which rotation apply to the module
            self.modules[-1].child_module = 0

            # update modules count
            self.num_modules = len(self.modules)

    def remove_module(self):
        """ Remove a module randomly (only if more than 2 modules available) """
        if len(self.modules) > MIN_NUM_MODULES:
            # select from which position remove the module
            idx = random.choice(range(self.num_modules))

            self.modules.pop(idx)

            # update modules IDs
            for i in range(idx, len(self.modules)):
                self.modules[i].order = i + 1
                self.modules[i].child_module = self.modules[i].order + 1

            # reset the child of last module, which is not attached to anything
            # on the contrary, the attached face is kept to preserve which rotation apply to the module
            self.modules[-1].child_module = 0

            # update modules count
            self.num_modules = len(self.modules)

    # ==== SIMULATION INPUT && LOGGING UTILS ==== #
    def string_input(self):
        """ Generate a string of parameters that are given to the simulator
            which builds and tests the particular robot configuration.
        """
        input_str = ''

        for module in self.modules:
            input_str += '{} - {} - {} - '.format(module.order,
                                                  module.child_module,
                                                  module.attached_face)

            # NOTE: rotation parameter no more used but kept
            # to avoid changing simulation input function TODO: update the code removing it
            input_str += '{} - {} - {} - {} - {} -- '.format(
                self.type_table[module.type]['freq'],
                self.type_table[module.type]['amp'],
                self.type_table[module.type]['phase'],
                module.rotation,
                module.stiffness
            )

        return input_str.strip()

    def simulate(self):
        fitnesses = []

        # select from which initial position the robot should be simulated
        for init_robot_pos in range(3):
            # Note: it is not necessary to run multiple times
            # when the simulation does not take into account the noise,
            # since in that case results are deterministic
            for _ in range(self.robot_tests):
                try:
                    exec_string = '{} {} {} {} {} {}'.format(self.simulation_path,
                                                             self.noise_type,
                                                             self.noise_level,
                                                             self.sim_seed,
                                                             init_robot_pos,
                                                             self.string_input())
                    c_proc = subprocess.run(exec_string.split(' '), capture_output=True)

                    # read the fitness from the results of simulation app
                    # Note: we assume that the latest output value corresponds to the
                    # fitness and that is separated by a space from the remaining part
                    fitnesses.append(float(c_proc.stdout.decode("utf-8").strip().split(' ')[-1]))
                except:
                    raise Exception('An error occurred during the simulation!\n'
                                    + 'Please run this {} and check the result'.format(exec_string))

        # take average avoiding division by zero (not reachable)
        # avg = reduce(lambda a, b: a + b, fitnesses) / len(fitnesses) if len(fitnesses) > 0 else 0.0
        #
        # print("Average fitness is " + str(avg))
        min_fit = min(fitnesses) if len(fitnesses) > 0 else 0.0
        # print("Min fitness is " + str(min_fit))

        return min_fit

    def simulate_bench(self):
        """ Benchmark function to test history functioning. """
        return (len(self.modules) * self.type_table[self.modules[0].type]['freq'] +
                 sum([m.attached_face * m.stiffness + self.type_table[m.type]['phase']
                      for m in self.modules]))

    def generate_trajectory(self, trajectory_file):
        """ Simulate a robt

        :param trajectory_file:
        :return:
        """
        try:
            # simulate robot from position 0
            exec_string = '{} {} {} {} {} {} {}'.format(self.tracker_path,
                                                        'in',
                                                        trajectory_file,
                                                        self.noise_type,
                                                        self.noise_level,
                                                        self.sim_seed,
                                                        self.string_input())
            subprocess.run(exec_string.split(' '), capture_output=True)
        except:
            print('Error! Trajectory not generated!')

    def change_stiff(self):
        # change stiffness of whole robot
        self.stiff_idx = random.randint(0, self.max_stiffness)
        self.stiffness = self.stiff_table[self.stiff_idx]

        for module in self.modules:
            module.stiffness = self.stiffness

    def plot_robot(self, folder, filename='graph.png'):
        """ Plot individual tree """
        graph = pgv.AGraph()
        for module in self.modules:
            for i, child in enumerate(module.child_codes):
                graph.add_edge(module.order, child)
        graph.layout(prog='dot')
        graph.draw(os.path.join(folder, filename))

    def representation(self):
        return self.string_input()

    # =========== OUTPUT UTILS =========== #
    def write_input(self, filename):
        """ Store robot information to be used as input for the simulation

        :param file:
        :return:
        """
        with open(filename, 'w') as out_file:
            for module in self.modules:
                out_file.write(module.get_hr_repr(self.type_table))
            out_file.write('fitness: {}\n'.format(self.fitness.values[0]))

# ==================================== #


# =========== EA OPERATORS =========== #
def evaluate_robot(robot):
    return float(robot.simulate()),


def crossover_robots(robot1, robot2):
    return robot1, robot2


def SUS(weights, N=1):
    # N := num individual to select
    # F := total fitness of Population
    # P := distance between the pointers (F/N)
    F = np.sum(weights)
    P = F / N
    # Start := random number between 0 and P
    start = random.uniform(0, P)
    pointers = [start + i * P for i in range(N)]

    return RWS(weights, pointers)


def RWS(weights, points):
    keep = []
    for p in points:
        i = 0
        while np.sum(weights[:i + 1]) < p:
            i += 1
        keep.append(i)
    return keep


def mutate_robot(robot):
    if random.random() < robot.p_global_mut:
        # work globally
        mutation_type = SUS(robot.weights['global'])[0]

        if mutation_type == 0:
            robot.add_module(robot.modules[0].stiffness)
        elif mutation_type == 1:
            robot.remove_module()
        else:
            # change stiffness of whole robot
            robot.change_stiff()

    # work locally on module
    for module in robot.modules:
        if random.random() < robot.p_local_mut:
            mutation_type = SUS(robot.weights['local'])[0]

            if mutation_type == 0:
                robot.switch_rotation(module.order)
            else:
                robot.switch_type(module.order)

    return robot,


def mutate_robot_per_module(robot):
    if random.random() < robot.p_global_mut:
        # work globally
        mutation_type = SUS(robot.weights['global'])[0]

        if mutation_type == 0:
            # randomly choose stiffness
            stiff_idx = random.randint(0, robot.max_stiffness)
            stiffness = robot.stiff_table[stiff_idx]
            robot.add_module(stiffness, stiff_idx)
        elif mutation_type == 1:
            robot.remove_module()

    # work locally on each module
    for module in robot.modules:
        if random.random() < robot.p_local_mut:
            mutation_type = SUS(robot.weights['local'])[0]

            if mutation_type == 0:
                robot.switch_rotation(module.order)
            elif mutation_type == 1:
                robot.switch_type(module.order)
            else:
                # change stiffness of single module
                module.stiff_idx = random.randint(0, robot.max_stiffness)
                module.stiffness = robot.stiff_table[module.stiff_idx]

    return robot,
