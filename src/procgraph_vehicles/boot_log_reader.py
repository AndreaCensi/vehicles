from procgraph import Block, Generator, simple_block, COMPULSORY
 
@simple_block 
def extract_from_extra(observations, fieldname=COMPULSORY):
    extra = observations['extra'].item()
    return extra[fieldname]

# TODO: move to BootOlympics

class BOLogReader(Generator):
    ''' 
        Reads the Vehicles log format (YAML)
    '''
    Block.alias('boot_log_reader')
    Block.output('observations')
    Block.config('logdir', 'BootOlympics logdir')
    Block.config('id_robot', 'Name of the robot')
    Block.config('id_episode', 'If given, select a specific episode', default="")
    Block.config('read_extra', 'Load the extra information.', default=True)
    
    def init(self): 
        from bootstrapping_olympics.logs import LogIndex
        index = LogIndex()
        index.index(self.config.logdir)
                
        def go():
            if not self.config.id_episode:
                for obs in index.read_all_robot_streams(self.config.id_robot,
                                                    read_extra=self.config.read_extra):
                    yield obs['timestamp'], obs
            else:
                for obs in index.read_robot_episode(self.config.id_robot,
                                                    id_episode=self.config.id_episode,
                                                    read_extra=self.config.read_extra):
                    yield obs['timestamp'], obs
        
        self.iterator = go()
        
        self._load_next()
        
    def _load_next(self):
        try:            
            self.next_timestamp, self.next_value = self.iterator.next()
            self.has_next = True
        except StopIteration:
            self.has_next = False
        
    def next_data_status(self): 
        if self.has_next:
            return (True, self.next_timestamp)
        else:
            return (False, None)

    def update(self):
        if not self.has_next:
            return # XXX: error here?
            
        self.set_output(0, value=self.next_value, timestamp=self.next_timestamp)
    
        self._load_next()
        
    def finish(self):
        pass
         
        
