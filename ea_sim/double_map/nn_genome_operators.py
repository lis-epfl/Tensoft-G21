import copy

from itertools import count


class NNGenomeOperators(object):
    def __init__(self, neat_config):
        super().__init__()
        self.genome_indexer = count(1)
        self.neat_config = neat_config
        self.last_id = 0
        self.genome_type = self.neat_config.genome_type
        self.genome_config = self.neat_config.genome_config
        self.last_node_id = 0

    def get_last_ids(self):
        return self.last_id, self.last_node_id

    def new_genome(self):
        gid = next(self.genome_indexer)
        genome = self.genome_type(gid)
        genome.id = gid
        genome.configure_new(self.genome_config)
        self.last_id = gid
        self.last_node_id = max(self.last_node_id, max(genome.nodes))

        return genome

    def crossover(self, parent1, parent2):
        gid = next(self.genome_indexer)
        child = self.genome_type(gid)
        child.id = gid
        child.configure_crossover(parent1, parent2, self.genome_config)
        self.last_id = gid
        self.last_node_id = max(self.last_node_id, max(child.nodes))

        return child

    def mutate(self, genome):
        gid = next(self.genome_indexer)
        genome.key = gid
        genome.id = gid
        genome.mutate(self.genome_config)
        self.last_id = gid
        self.last_node_id = max(self.last_node_id, max(genome.nodes))

        return genome
