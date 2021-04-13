import fcntl
import numpy as np
import random
import re
import subprocess

from math import isclose


class ModuleGR:
    def __init__(self, order, child_module, attached_face, freq, amplitude, phase, rotation, stiffness):
        self.order = order
        self.child_module = child_module
        # the module face where the child is attached
        self.attached_face = attached_face
        self.freq = freq
        self.amplitude = amplitude
        self.phase = phase
        # NOTE: rotation parameter no more used but kept
        # to avoid changing simulation input function TODO: update the code removing it
        self.rotation = rotation
        self.stiffness = stiffness

    def __eq__(self, other):
        """ Check whether this module is equivalent
            (has the same values) to the other.

        :param other:
        :return:
        """
        return isinstance(other, self.__class__) and self.order == other.order \
            and self.order == other.order \
            and self.child_module == other.child_module \
            and self.attached_face == other.attached_face \
            and self.freq == other.freq \
            and self.amplitude == other.amplitude \
            and self.phase == other.phase  \
            and self.rotation == other.rotation \
            and isclose(self.stiffness, other.stiffness, rel_tol=1e-05)

    def get_hr_repr(self):
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
            self.freq,
            self.amplitude,
            self.phase,
            self.rotation,
            self.stiffness
        )

        return output

    def get_info_repr(self):
        """ Get a representation of this module

        :param type_table:
        :return:
        """
        return {
            'order': self.order,
            'frequency': self.freq,
            'amplitude': self.amplitude,
            'phase': self.phase,
            'rot': self.rotation,
            'stiff': self.stiffness
        }


class RobotGR(list):
    def __init__(self, direction_path, rotation_angle_path, simulation_path, tracker_path,
                 sim_time, noise_type, noise_level, target_dist_bearing, id, num_faces,
                 num_modules, robot_tests, sim_seeds_seed, modules_conf):
        super().__init__()

        # simulation related parameters
        self.sim_time = sim_time
        self.direction_path = direction_path
        self.rotation_angle_path = rotation_angle_path
        self.simulation_path = simulation_path
        self.tracker_path = tracker_path
        self.noise_type = noise_type
        self.noise_level = noise_level

        self.target_dist_bearing = target_dist_bearing

        self.id = id
        self.num_faces = num_faces
        self.num_modules = num_modules
        self.modules = [None for _ in range(self.num_modules)]
        self._assign_parents(modules_conf)

        self.robot_tests = robot_tests
        if sim_seeds_seed is not None:
            random.seed(sim_seeds_seed)
            self.sim_seeds = [
                random.randint(0, 1000000)
                for _ in range(0, len(self.target_dist_bearing)*self.robot_tests)
            ]
            self.direction = self.get_direction()
        else:
            self.sim_seeds = None
            self.direction = None

        if self.rotation_angle_path is not None:
            self.rotation_angle = self.get_rotation_angle()
        else:
            self.rotation_angle = None

    def _assign_parents(self, modules_conf):
        """ Generate the modules that compose the robot and connect them together. """
        for i in range(self.num_modules):
            module_conf = modules_conf[i]

            # initialize a new module
            module = ModuleGR(module_conf['order'], module_conf['connectedModules'],
                              module_conf['connectedFaces'], module_conf['freq'],
                              module_conf['amplitude'], module_conf['phase'],
                              module_conf['rot'], module_conf['stiff'])

            self.modules[i] = module

    def __eq__(self, other):
        """ Check whether this robot is equivalent
            (has the same parameters and structure) to the other.

        :param other:
        :return:
        """
        if not isinstance(other, self.__class__) \
                or self.num_modules != other.num_modules:
            return False

        for a, b in zip(self.modules, other.modules):
            if a != b:
                return False

        return True

    def info(self):
        """ Return robot information to be logged. """
        return {
            'id': self.id,
            'num_modules': self.num_modules,
            'modules': [m.get_info_repr() for m in self.modules],
            'sim_string': self.string_input()
        }

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
                module.freq,
                module.amplitude,
                module.phase,
                module.rotation,
                module.stiffness
            )

        return input_str.strip()

    def get_direction(self):
        try:
            exec_string = '{} {} {} {} {} {}'.format(self.direction_path,
                                                     self.noise_type,
                                                     self.noise_level,
                                                     self.sim_seeds[0],
                                                     0,  # robot initial position
                                                     self.string_input())

            c_proc = subprocess.run(exec_string.split(' '), capture_output=True)

            # read the fitness from the results of simulation app
            # Note: we assume that the latest output value corresponds to the
            # fitness and that is separated by a space from the remaining part
            exec_out = re.split(' |\n', c_proc.stdout.decode("utf-8").strip())

            return int(exec_out[-1])
        except:
            raise Exception('An error occurred during the simulation!\n'
                                + 'Please run this {} and check the result'.format(exec_string))

    def get_rotation_angle(self):
        try:
            exec_string = '{} {} {} {}'.format(self.rotation_angle_path,
                                               self.direction,
                                               0,  # robot initial position
                                               self.string_input())

            c_proc = subprocess.run(exec_string.split(' '), capture_output=True)

            # read the fitness from the results of simulation app
            # Note: we assume that the latest output value corresponds to the
            # fitness and that is separated by a space from the remaining part
            exec_out = re.split(' |\n', c_proc.stdout.decode("utf-8").strip())

            return exec_out[-1]
        except:
            raise Exception('An error occurred during the simulation!\n'
                            + 'Please run this {} and check the result'.format(exec_string))

    def simulate(self, controller_path, invert_distance=True, m_id=None, data_file_paths=None):
        distances = []
        target_positions = []
        robot_init_pos = None

        for target_index in range(len(self.target_dist_bearing)):
            target_dist_bearing = self.target_dist_bearing[target_index]

            distances.append([])
            target_positions.append(None)

            for i in range(0, self.robot_tests):
                sim_index = target_index * self.robot_tests + i

                try:
                    exec_string = '{} {} {} {} {} {} {}'.format(self.simulation_path,
                                                                self.sim_time,
                                                                self.noise_type,
                                                                self.noise_level,
                                                                self.sim_seeds[sim_index],
                                                                controller_path,
                                                                self.direction)

                    if 'SGR' in self.simulation_path and self.rotation_angle is not None:
                        exec_string = '{} {}'.format(exec_string, self.rotation_angle)

                    exec_string = '{} {} {}'.format(exec_string,
                                                    target_dist_bearing[0],
                                                    target_dist_bearing[1])

                    if data_file_paths is not None:
                        exec_string = '{} {}'.format(exec_string,
                                                     data_file_paths[sim_index])

                    exec_string = '{} {} {}'.format(exec_string,
                                                    0,  # robot initial position
                                                    self.string_input())

                    c_proc = subprocess.run(exec_string.split(' '), capture_output=True)

                    # read the the results of simulation app
                    exec_out = re.split(' |\n', c_proc.stdout.decode("utf-8").strip())

                    distances[target_index].append(float(exec_out[-1]))

                    if target_positions[target_index] is None:
                        target_positions[target_index] = [
                            exec_out[-10],
                            exec_out[-9]
                        ]

                    if robot_init_pos is None:
                        robot_init_pos = [
                            exec_out[-14],
                            exec_out[-13]
                        ]
                except:
                    raise Exception('An error occurred during the simulation!\n'
                                    + 'Please run this {} and check the result'.format(exec_string))

        distance_per_target = [np.mean(target_dists) for target_dists in distances if len(target_dists) > 0]
        avg_dist = np.mean(distance_per_target) if len(distance_per_target) > 0 else None
        max_dist = max(distance_per_target) if len(distance_per_target) > 0 else None

        fitness = avg_dist + 0.5 * (max_dist - avg_dist) \
            if avg_dist is not None and max_dist is not None \
            else None
        self.append_fitness(controller_path, fitness, distances, target_positions, robot_init_pos, m_id)

        if invert_distance:
            fitness = 1.0 / (fitness + 1e-10) if fitness is not None else None

        return fitness

    # =========== OUTPUT UTILS =========== #
    def write_input_sim(self, filename):
        """ Store robot information to be used as input for the simulation
        :param file:
        :return:
        """
        exec_string = '{} {} {} {} {} {} {}'.format(self.simulation_path,
                                                    self.sim_time,
                                                    self.noise_type,
                                                    self.noise_level,
                                                    'sim_seed',
                                                    'controller_path',
                                                    'direction')

        if 'SGR' in self.simulation_path:
            exec_string = '{} {}'.format(exec_string, 'rotation_angle')

        exec_string = '{} {} {} {} {}'.format(exec_string,
                                              'target_dist',
                                              'target_bearing',
                                              0,  # robot initial position
                                              self.string_input())

        with open(filename, 'w') as out_file:
            out_file.write(exec_string+'\n')

    def write_input_vis(self, filename):
        """ Store robot information to be used as input for the visual simulation
        :param filename:
        :return:
        """
        with open(filename, 'w') as out_file:
            for module in self.modules:
                out_file.write(module.get_hr_repr())

    def append_fitness(self, filename, fit_val, fitnesses, target_positions, robot_init_pos, m_id=None):
        output_string = ''

        output_string += 'fitness: {}\n'.format(fit_val)

        if m_id is not None:
            output_string += 'morphology_id: {}\n'.format(m_id)
            output_string += 'num_modules: {}\n'.format(self.num_modules)

        output_string += 'robot_init_pos: {} {}\n'.format(
            robot_init_pos[0].rjust(22, ' '),
            robot_init_pos[1].rjust(22, ' ')
        )
        output_string += 'direction: {}\n'.format(self.direction)

        if 'SGR' in self.simulation_path and self.rotation_angle is not None:
            output_string += 'rotation_angle: {}\n'.format(self.rotation_angle)

        output_string += 'sim-fit: \n'
        for i in range(len(self.target_dist_bearing)):
            sim_seeds = self.sim_seeds[i * self.robot_tests: (i + 1) * self.robot_tests]
            str_fit_vals = ['{:7.4f}'.format(fitness_val) for fitness_val in fitnesses[i]]
            output_string += '\tdist: {:3d}\tbearing: {:8.4f}\tpos: {} {}\tsim_seeds: {}\tfits: {} \n'.format(
                self.target_dist_bearing[i][0],
                self.target_dist_bearing[i][1],
                target_positions[i][0].rjust(22, ' '),
                target_positions[i][1].rjust(22, ' '),
                ', '.join([str(s) for s in sim_seeds]),
                ', '.join(str_fit_vals)
            )

        with open(filename, 'a') as c_file:
            fcntl.flock(c_file, fcntl.LOCK_EX)
            c_file.write(output_string)
            fcntl.flock(c_file, fcntl.LOCK_UN)
