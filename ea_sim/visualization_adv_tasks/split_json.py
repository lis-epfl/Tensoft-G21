import argparse
import os
import subprocess


def save_chunk(base_filename, chunk_num, chunk_content):
    filename = base_filename.replace('.json', '_{}.json'.format(chunk_num))

    with open(filename, 'w') as out_file:
        if not chunk_content.startswith('['):
            chunk_content = '[' + chunk_content[1:]
        else:
            chunk_content = '[ ' + chunk_content[1:]

        if not chunk_content.endswith(']'):
            chunk_content = chunk_content + ' ]'
        else:
            chunk_content = chunk_content[:-1] + ' ]'

        out_file.write(chunk_content)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Script for splitting json evo file in chunks.')
    parser.add_argument('json_file', metavar='json_file', type=str, nargs='?',
                        help='json file to split')
    parser.add_argument('--bytes-to-read', metavar='bytes-to-read', type=int, action='store',
                        default=209715200, help='number of bytes to read at each iteration')
    parser.add_argument('--objs-per-chunk', metavar='objs-per-chunk', type=int, action='store',
                        default=1, help='number of objects to split after which')
    parser.add_argument('--objs-sep', metavar='objs-sep', type=str, action='store',
                        default='}]}', help='objects separator')
    parser.add_argument('--owner', metavar='owner', type=str, action='store', nargs='+',
                        default=None, help='User and group for chown')

    args = parser.parse_args()

    input_file = args.json_file
    bytes_to_read = args.bytes_to_read
    objs_per_chunk = args.objs_per_chunk
    objs_separator = args.objs_sep
    owner = args.owner

    with open(input_file, 'rb') as in_file:
        eof = False
        remaining = ''
        chunk_counter = 0

        while not eof:
            print('Chunk #{}'.format(chunk_counter))
            to_be_saved = ''
            objs_counter = 0

            while not eof and objs_counter < objs_per_chunk:
                if remaining != '':
                    in_chunk = remaining
                    remaining = ''
                else:
                    in_chunk = str(in_file.read(bytes_to_read), 'utf-8')

                if in_chunk == '':
                    eof = True
                    if to_be_saved != '' and objs_counter > 0:
                        save_chunk(input_file, chunk_counter, to_be_saved)
                else:
                    in_chunk_objs_num = in_chunk.count(objs_separator)
                    objs_counter += in_chunk_objs_num

                    if objs_counter >= objs_per_chunk:
                        split_occurrence = in_chunk_objs_num - (objs_counter-objs_per_chunk)
                        groups = in_chunk.split(objs_separator)

                        to_be_saved += (objs_separator.join(groups[0:split_occurrence])+objs_separator)
                        save_chunk(input_file, chunk_counter, to_be_saved)

                        to_be_saved = ''
                        remaining = objs_separator.join(groups[split_occurrence:])
                    else:
                        to_be_saved += in_chunk

            chunk_counter += 1

    if owner is not None and len(owner) == 2:
        try:
            dir_path = os.path.dirname(input_file)
            dir_path = dir_path if len(dir_path) > 0 else os.path.curdir

            exec_string = 'chown -R {}:{} {}'.format(owner[0], owner[1], dir_path)

            c_proc = subprocess.run(exec_string.split(' '), capture_output=True)
        except:
            raise Exception('An error occurred during the owner setting')
