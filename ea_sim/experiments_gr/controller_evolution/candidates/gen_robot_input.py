import argparse
import json
import os


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for generating GR input')
    parser.add_argument('robot_morphology', metavar='robot_morphology', type=str, nargs='?',
                        default='robot.txt',
                        help='the robot morphology file')
    parser.add_argument('input_template', metavar='input_template', type=str, nargs='?',
                        default='robot_template.json',
                        help='the input template file')
    parser.add_argument('robot_id', metavar='robot_id', type=int, nargs='?',
                        default=0,
                        help='robot id')
    parser.add_argument('seed', metavar='seed', type=int, nargs='?',
                        default=27,
                        help='seed')
    parser.add_argument('num_sims', metavar='num_sims', type=int, nargs='?',
                        default=50000,
                        help='number of evaluations')
    args = parser.parse_args()

    # open template
    with open(args.input_template, 'r') as template_file:
        input_obj = json.load(template_file)

    # set robot id
    input_obj['robot']['id'] = args.robot_id

    # parse and set robot morphology
    num_modules = 0
    with open(args.robot_morphology, 'r') as robot_morph:
        for line in robot_morph:
            elements = line.split(' ')

            if elements[0] != 'fitness:':
                num_modules += 1
                module_conf = {
                    "order": int(elements[1]),
                    "connectedModules": int(elements[3]),
                    "connectedFaces": int(elements[5]),
                    "freq": float(elements[7]),
                    "amplitude": float(elements[9]),
                    "phase": float(elements[11]) if elements[11] != '0' else 0,
                    "rot": int(elements[13]),
                    "stiff": float(elements[15])
                }
                input_obj['robot']['modules_conf'] += [module_conf]
            else:
                # set target distance and bearing
                t_distance = 45  # int(floor(0.85*float(elements[1])))
                bearings = [-1.57, -0.78, 0.78, 1.57]

                for bearing in bearings:
                    target = [t_distance, bearing]
                    input_obj['target_dist_bearing'] += [target]

    # set number of modules
    input_obj['robot']['num_modules'] = num_modules

    # set seed
    input_obj['seeds'] += [args.seed]

    # set num_sims
    input_obj['max_num_sims'] = args.num_sims

    # adjust result dir path
    input_obj['result_dir'] = '{}_{}'.format(input_obj['result_dir'], args.robot_id)

    with open(os.path.join('..', 'robot_{}.json'.format(args.robot_id)), 'w') as gr_input:
        json.dump(input_obj, gr_input, indent=4)
