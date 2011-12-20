from procgraph import Block, Generator
import yaml


try:
    # Try to load the C bindings
    from yaml import CLoader as Loader
except ImportError:  # XXX
    print('Error importing C bindings for yaml.')
    from yaml import Loader


class YAMLLogReader(Generator):
    '''
        Reads the Vehicles log format (YAML)
    '''
    Block.alias('yaml_log_reader')
    Block.output('state')
    Block.config('file', 'YAML file to read')

    def init(self):
        self.f = open(self.config.file)
        self.iterator = yaml.load_all(self.f, Loader=Loader)
        self._load_next()

    def _load_next(self):
        try:
            self.next_value = self.iterator.next()
            self.next_timestamp = self.next_value['timestamp']
            self.has_next = True
        except StopIteration:
            self.has_next = False

    def next_data_status(self):
        if self.has_next:
            return ('state', self.next_timestamp)
        else:
            return (False, None)

    def update(self):
        if not self.has_next:
            return  # XXX: error here?

        self.set_output(0, value=self.next_value,
                        timestamp=self.next_timestamp)

        self._load_next()

    def finish(self):
        self.f.close()
