import neat
import os


class Controller(object):
    def __init__(self, c_id, seed, genome, config):
        super().__init__()

        self.c_id = c_id
        self.seed = seed
        self.genome = genome
        self.config = config
        self.nn = neat.nn.FeedForwardNetwork.create(self.genome, self.config)

    def save(self, dir_path, check_file_existence=False):
        if len(self.nn.node_evals) > 0:
            file_path = os.path.join(dir_path, 'controller_' + str(self.seed) + '_' + str(self.c_id) + '.txt')

            if not check_file_existence or not os.path.exists(file_path):
                with open(file_path, 'w+') as out:
                    out.write(str(len(self.nn.input_nodes))+" "+" ".join([str(i) for i in self.nn.input_nodes])+"\n")
                    out.write(str(len(self.nn.output_nodes))+" "+" ".join([str(o) for o in self.nn.output_nodes]) + "\n")

                    for node, _, _, bias, _, links in self.nn.node_evals:
                        out.write('{} '.format(node) + '{0:.40f}'.format(bias) + ' {}'.format(len(links)))
                        for link in links:
                            out.write(' {} '.format(link[0]) + '{0:.40f}'.format(link[1]))
                        out.write('\n')

            return file_path
        else:
            return None
