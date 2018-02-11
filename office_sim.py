from P_MAS_TG.ts import MotionFts, ActionModel, MotActModel
from P_MAS_TG.planner import ltl_planner

# export PYTHONPATH=$PYTHONPATH:/to/your/P_MAS_TG


import time


##############################
# motion FTS
ap = {'p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7', 'p8', 'p9'}

regions = {   (15, -45, 5): set(['p1',]),
              (25, -15, 5): set(['p2',]),
              (50, -10, 5): set(['p3',]),
              (45, -25, 5): set(['p4',]),
              (60, -15, 5): set(['p5',]),
              (60, -35, 5): set(['p6',]),
              (33, -43, 5): set(['p7',]),
              (45, -45, 5): set(['p8',]),
              (60, -50, 5): set(['p9',]),
}

edges = [('p1', 'p2'),
         ('p2', 'p7'),
         ('p2', 'p4'),
         ('p4', 'p6'),
         ('p4', 'p8'),
         ('p6', 'p9'),
         ('p2', 'p3'),
         ('p3', 'p5'),         
]

robot_motion = MotionFts(regions, ap, 'office' )
robot_motion.set_initial((15, -45, 5))
robot_motion.add_un_edges_by_ap(edges, unit_cost = 0.1)


##############################
# action FTS
############# no action model
action = dict()
############# with action
robot_action = ActionModel(action)


##############################
# complete robot model
robot_model = MotActModel(robot_motion, robot_action)



##############################
# specify tasks
########## only hard
# robot 1
# hard_task = '<>(p1 && <> (p4 && <> p8)) && ([]<> p6) && ([]<> p9)'
# robot 2
# hard_task = '([]<> (p1 && <> p2)) && ([]<> p4)'
# robot 3
# hard_task = '([]<> (p3 && <> (p6 || p8)))'
# robot 4
hard_task = '([]<> p7) && ([]<> p8) && ([]<> p5)'

soft_task = None

print 'robot_task: %s' %hard_task
##############################
# set planner
robot_planner = ltl_planner(robot_model, hard_task, soft_task)

# synthesis
start = time.time()
robot_planner.optimal(10,'static')

print 'full construction and synthesis done within %.2fs \n' %(time.time()-start)
