from P_MAS_TG.ts import MotionFts, ActionModel, MotActModel
from P_MAS_TG.planner import ltl_planner

# export PYTHONPATH=$PYTHONPATH:/to/your/P_MAS_TG


import time


##############################
# motion FTS
ap = {'r1', 'r2', 'r3', 'r4', 'r5', 'r6', 'r', 'b'}
# +-----+-----+-----+
# | r4,r| r5,b| r6,b|
# +-----+-----+-----+
# | r1,r| r2,b| r3,r|
# +-----+-----+-----+
regions = {   (0, 0, 1): set(['r1', 'r']),
              (1, 0, 1): set(['r2', 'b']),
              (2, 0, 1): set(['r3', 'r']),
              (0, 1, 1): set(['r4', 'r']),
              (1, 1, 1): set(['r5', 'b']),
              (2, 1, 1): set(['r6', 'b']),
}

edges = [((0, 0, 1), (1, 0, 1)),
         ((1, 0, 1), (2, 0, 1)), 
         ((0, 1, 1), (1, 1, 1)),          
         ((1, 1, 1), (2, 1, 1)),
         ((0, 0, 1), (0, 1, 1)),
         ((1, 0, 1), (1, 1, 1)),
         ((2, 0, 1), (2, 1, 1)),          
]

robot_motion = MotionFts(regions, ap, 'office' )
robot_motion.set_initial((0, 0, 1))
robot_motion.add_un_edges(edges, unit_cost = 0.1)


##############################
# action FTS
############# no action model
#action = dict()
############# with action
action = { 'pick': (100, 'r', set(['pick'])),
           'drop': (50, 'b', set(['drop']))
}


robot_action = ActionModel(action)


##############################
# complete robot model
robot_model = MotActModel(robot_motion, robot_action)



##############################
# specify tasks
########## only hard
# hard_task = '<>(r1 && <> (r2 && <> r6)) && (<>[] r6)'
#hard_task = '(<>(pick && <> drop)) && ([]<> r3) && ([]<> r6)'
#soft_task = None


########## soft and hard
hard_task = '(([]<> r3) && ([]<> r4))'
soft_task = '([]! b)'



##############################
# set planner
robot_planner = ltl_planner(robot_model, hard_task, soft_task)

# synthesis
start = time.time()
robot_planner.optimal(10,'static')

print 'full construction and synthesis done within %.2fs \n' %(time.time()-start)
