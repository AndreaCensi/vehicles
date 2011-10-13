from . import all_robots, get_robot, all_agents, get_agent, logger
import sys

def add_robot_f(f, id_robot):
    def test_caller():
        robot = get_robot(id_robot)
        wrap_with_desc(f, (id_robot, robot), agent=None, robot=robot)

    name = '%s-%s' % (f.__name__, id_robot)
    test_caller.__name__ = name
    test_caller.robot = id_robot

    module = sys.modules[f.__module__]
    if name in module.__dict__:
        raise Exception('Already created test %r.' % name)
    module.__dict__[name] = test_caller

def for_all_robots(f):
    for id_robot in all_robots():
        add_robot_f(f, id_robot)
        
def add_agent_f(f, id_agent):
    def test_caller():
        agent = get_agent(id_agent)
        wrap_with_desc(f, (id_agent, agent), agent=agent, robot=None)
        
    name = '%s-%s' % (f.__name__, id_agent)
    test_caller.__name__ = name
    test_caller.agent = id_agent

    module = sys.modules[f.__module__]
    if name in module.__dict__:
        raise Exception('Already created test %r.' % name)
    module.__dict__[name] = test_caller
 
def for_all_agents(f):
    for id_agent in all_agents():
        add_agent_f(f, id_agent)

def add_pair_f(f, id_robot, id_agent):
    def test_caller():
        agent = get_agent(id_agent)
        robot = get_robot(id_robot)
        wrap_with_desc(f, (id_agent, agent, id_robot, robot),
                       agent=agent, robot=robot)
        
    name = '%s-%s-%s' % (f.__name__, id_agent, id_robot)
    test_caller.__name__ = name
    module = sys.modules[f.__module__]
    test_caller.agent = id_agent
    test_caller.robot = id_robot
    if name in module.__dict__:
        raise Exception('Already created test %r.' % name)
    module.__dict__[name] = test_caller 

def for_all_pairs(f):
    for id_agent in all_agents():
        for id_robot in all_robots():
            add_pair_f(f, id_robot, id_agent)

def wrap_with_desc(function, arguments, agent=None, robot=None):
    ''' Calls function with arguments, and writes debug information
        if an exception is detected. '''
    
    try:
        function(*arguments)
    except:
        msg = ('Error detected when running test (%s); displaying debug info.' 
               % function.__name__)
        if robot is not None:
            msg += '\nRobot: %s' % robot
            msg += '\n Obs spec: %s' % robot.get_spec().get_observations()
            msg += '\n Cmd spec: %s' % robot.get_spec().get_commands()
        if agent is not None:
            msg += '\nAgent: %s' % agent
            
        logger.error(msg)
        raise 
     
    
    
