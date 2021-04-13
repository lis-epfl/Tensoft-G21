import gzip
import pickle

from neat import StatisticsReporter


class StatisticsReporterGR(StatisticsReporter):
    def __init__(self, generation_interval=100, filename_prefix='neat-statistics-',
                 most_fit_genomes=None, generation_statistics=None):
        StatisticsReporter.__init__(self)
        self.generation_interval = generation_interval
        self.filename_prefix = filename_prefix

        if most_fit_genomes is not None:
            self.most_fit_genomes = most_fit_genomes
        if generation_statistics is not None:
            self.generation_statistics = generation_statistics

        self.current_generation = None
        self.last_generation_checkpoint = -1

    def start_generation(self, generation):
        self.current_generation = generation

    def end_generation(self, config, population, species_set):
        if self.generation_interval is not None:
            dg = self.current_generation - self.last_generation_checkpoint
            if dg >= self.generation_interval:
                self.save_checkpoint(self.current_generation, self.most_fit_genomes, self.generation_statistics)
                self.last_generation_checkpoint = self.current_generation

    def save_checkpoint(self, generation, most_fit_genomes, generation_statistics):
        """ Save the current statistics. """
        filename = '{0}{1}'.format(self.filename_prefix, generation)
        print("Saving NEAT statistics to {0}".format(filename))

        with gzip.open(filename, 'w', compresslevel=5) as f:
            data = (most_fit_genomes, generation_statistics)
            pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)

    @staticmethod
    def restore_checkpoint(filename):
        """Resumes the simulation from a previous saved point."""
        with gzip.open(filename) as f:
            most_fit_genomes, generation_statistics = pickle.load(f)
            return most_fit_genomes, generation_statistics
