from procgraph import Block, simple_block, COMPULSORY
import numpy as np

import procgraph_boot  # some things moved here @UnusedImport


@simple_block
def extract_field(ob, field=COMPULSORY):
    try: 
        return ob[field]
    except Exception as e:
        msg = 'Could not find field %r: %s.' % (field, e)
        if isinstance(ob, dict):
            msg += ('I know %s.' % ob.keys())
        if isinstance(ob, np.ndarray):
            msg += 'Available: %s' % ob.dtype
        raise ValueError(msg)
    

@simple_block
def extract_timeinfo(state):
    timestamp = state['timestamp']
#    id_episode = state['id_episode']
#    return "Episode %20s %6.2fs" % (id_episode, timestamp)
    return "%6.2fs" % (timestamp)

class use_simulation_time(Block):
    Block.input('state')
    Block.output('state')

    def update(self):
        state = self.input.state
        timestamp = state['timestamp']
        self.set_output(0, state, timestamp)
