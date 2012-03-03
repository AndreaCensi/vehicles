from procgraph import Block, simple_block, COMPULSORY
import numpy as np


@simple_block
def extract_from_extra(observations, fieldname=COMPULSORY):
    extra = observations['extra'].item()
    return extra[fieldname]


@simple_block
def extract_sensels(state):
    return np.array(state['observations'])


@simple_block
def extract_commands(state):
    return np.array(state['commands'])


@simple_block
def extract_field(dictionary, field=COMPULSORY):
    if not field in dictionary:
        msg = ('Could not find field %r; I know %s.'
               % (field, dictionary.keys()))
        raise ValueError(msg)

    return dictionary[field]


@simple_block
def extract_timeinfo(state):
    timestamp = state['timestamp']
    id_episode = state['id_episode']
    return "Episode %20s %6.2fs" % (id_episode, timestamp)


class use_simulation_time(Block):
    Block.input('state')
    Block.output('state')

    def update(self):
        state = self.input.state
        timestamp = state['timestamp']
        self.set_output(0, state, timestamp)
