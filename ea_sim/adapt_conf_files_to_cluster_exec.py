import argparse
import copy
import json
import os
import re
import shutil


class MyJSONEncoder(json.JSONEncoder):
    def __init__(self, *args, **kwargs):
        super(MyJSONEncoder, self).__init__(*args, **kwargs)
        self.current_indent = 0
        self.current_indent_str = ""

    def encode(self, obj):
        if isinstance(obj, (list, tuple)):
            primitives_only = True
            for item in obj:
                if isinstance(item, (list, tuple, dict)):
                    primitives_only = False
                    break
            output = []
            if primitives_only:
                for item in obj:
                    output.append(json.dumps(item))
                return "[" + ", ".join(output) + "]"
            else:
                self.current_indent += self.indent
                self.current_indent_str = "".join([" " for x in range(self.current_indent)])
                for item in obj:
                    output.append(self.current_indent_str + self.encode(item))
                self.current_indent -= self.indent
                self.current_indent_str = "".join([" " for x in range(self.current_indent)])
                return "[\n" + ",\n".join(output) + "\n" + self.current_indent_str + "]"
        elif isinstance(obj, dict):
            output = []
            self.current_indent += self.indent
            self.current_indent_str = "".join([" " for x in range(self.current_indent)])
            for key, value in obj.items():
                output.append(self.current_indent_str + json.dumps(key) + ": " + self.encode(value))
            self.current_indent -= self.indent
            self.current_indent_str = "".join([" " for x in range(self.current_indent)])
            return "{\n" + ",\n".join(output) + "\n" + self.current_indent_str + "}"
        else:
            return json.dumps(obj)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for generating experiment configuration files suitable '
                                                 'to cluster execution')
    parser.add_argument('conf_files_dirs', metavar='conf_files_dirs', type=str, nargs='+',
                        help='list of folders containing the configuration files')
    parser.add_argument('--seeds', metavar='seeds', type=int, action='store', nargs='+',
                        default=None, help='list of evolution seeds')
    parser.add_argument('--num-sims', metavar='num_sims', type=int, action='store', nargs='?',
                        default=None, help='number of simulations (evaluations)')
    parser.add_argument('--user-path', metavar='user_path', type=str, action='store', nargs='?',
                        default='/data/zambrano', help='user path in cluster')
    parser.add_argument('--output-dir', metavar='output_dir', type=str, action='store', nargs='?',
                        default='./cluster_exps', help='the folder where the output files will be stored.')
    parser.add_argument('--output-subdirs', metavar='output_subdirs', type=str, action='store', nargs='+',
                        default=None, help='list of output sub-folders names for corresponding input folders')

    args = parser.parse_args()

    output_dir = args.output_dir
    os.makedirs(output_dir, exist_ok=True)

    if args.output_subdirs is not None and len(args.conf_files_dirs) != len(args.output_subdirs):
        print('The number of output sub-directories must match the number of input folders!')
        exit(0)

    for i in range(0, len(args.conf_files_dirs)):
        conf_files_dir = args.conf_files_dirs[i]

        if args.output_subdirs is not None:
            output_subdir = os.path.join(output_dir, args.output_subdirs[i])
        else:
            output_subdir = os.path.join(output_dir, conf_files_dir)
        os.makedirs(output_subdir, exist_ok=True)

        conf_files_paths = {
            f: os.path.join(conf_files_dir, f)
            for f in os.listdir(conf_files_dir)
            if re.match('.*\.json', f)
        }

        for conf_filename, conf_file_path in conf_files_paths.items():
            with open(conf_file_path) as conf_file:
                conf_obj = json.load(conf_file)

                if args.seeds is None:
                    seeds = conf_obj['seeds']
                else:
                    seeds = args.seeds

                for key, val in conf_obj.items():
                    if 'path' in key:
                        conf_obj[key] = val.replace('/home', args.user_path)
                    elif key == 'result_dir':
                        conf_obj[key] = val.replace('/home', args.user_path)\
                                           .replace('/results/', '/ea_sim/results/')

                for j in range(0, len(seeds)):
                    seed = seeds[j]

                    output_object = copy.deepcopy(conf_obj)
                    output_object['result_dir'] = '{}_{}'.format(conf_obj['result_dir'], j)
                    output_object['seeds'] = [seed]
                    if args.num_sims is not None:
                        output_object['max_num_sims'] = args.num_sims

                    output_file_path = os.path.join(
                        output_subdir,
                        '{}_scitas_{}.json'.format(conf_filename[:-5], j)
                    )

                    with open(output_file_path, 'w') as out_file:
                        out_file.write(json.dumps(output_object, cls=MyJSONEncoder, indent=4))

        for f in os.listdir(conf_files_dir):
            if re.match('neat-settings.*\.txt', f):
                shutil.copy(
                    os.path.join(conf_files_dir, f),
                    output_subdir
                )
