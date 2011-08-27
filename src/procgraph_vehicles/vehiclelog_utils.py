from procgraph import simple_block
from procgraph import Block


@simple_block
def extract_sensels(state):
    return state['observations']


@simple_block
def extract_commands(state):
    return state['commands']


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

