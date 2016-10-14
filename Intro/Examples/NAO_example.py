# -*- coding: utf-8 -*-

from ts import MotionFts, ActionModel, MotActModel
from planner import ltl_planner

import time

###############################
################# initialize  NAO_motion
colormap = {'ball':'red', 'obs':'black','basket':'yellow'}
symbols = colormap.keys()
WIDTH = 2400 # mm
HEIGHT = 2100 # mm
N = 6.0
RATE = 1/N
init_pose = (WIDTH*RATE,HEIGHT*RATE);
node_dict ={
			# lower three rooms
			(WIDTH*RATE,HEIGHT*RATE): set([]),
			(WIDTH*3*RATE,HEIGHT*RATE): set([]),
			(WIDTH*5*RATE,HEIGHT*RATE): set([]),
			# cooridor three parts
			(WIDTH*RATE,HEIGHT*3*RATE): set([]),
			(WIDTH*3*RATE,HEIGHT*3*RATE): set(['ball']),
			(WIDTH*5*RATE,HEIGHT*3*RATE): set([]),
			# upper three rooms
			(WIDTH*RATE,HEIGHT*5*RATE): set(['basket']),
			(WIDTH*3*RATE,HEIGHT*5*RATE): set([]),
			(WIDTH*5*RATE,HEIGHT*5*RATE): set([]),
			}
NAO_motion = MotionFts(node_dict, symbols, 'office')
NAO_motion.set_initial(init_pose)
edge_list = [ # 1st column
			 ((WIDTH*RATE,HEIGHT*RATE), (WIDTH*RATE,HEIGHT*3*RATE)),
			 ((WIDTH*RATE,HEIGHT*3*RATE), (WIDTH*RATE,HEIGHT*5*RATE)),
			 # 2nd row
			 ((WIDTH*RATE,HEIGHT*3*RATE), (WIDTH*3*RATE,HEIGHT*3*RATE)),
			 ((WIDTH*3*RATE,HEIGHT*3*RATE), (WIDTH*5*RATE,HEIGHT*3*RATE)),
			 # 2nd column
			 ((WIDTH*3*RATE,HEIGHT*RATE), (WIDTH*3*RATE,HEIGHT*3*RATE)),
			 ((WIDTH*3*RATE,HEIGHT*3*RATE), (WIDTH*3*RATE,HEIGHT*5*RATE)),
			 # 3rd column
			 ((WIDTH*5*RATE,HEIGHT*RATE), (WIDTH*5*RATE,HEIGHT*3*RATE)),
			 ((WIDTH*5*RATE,HEIGHT*3*RATE), (WIDTH*5*RATE,HEIGHT*5*RATE)),
			 ]
NAO_motion.add_un_edges(edge_list,unit_cost=0.1)
###############################
################# initialize  NAO_action
action_dict={
			 'pick': (100, 'ball', set(['pick'])),
			 'drop': (60, 'basket', set(['drop']))
			}
NAO_action = ActionModel(action_dict)
###############################
################# initialize  full NAO_model
NAO_model = MotActModel(NAO_motion, NAO_action)
###############################
############### define task
hard_spec = '([]<>pick) && ([](pick -> X(!pick U drop))) && ([](drop -> X(!drop U pick)))'
soft_spec = None
###############################
############## set planner
NAO_planner = ltl_planner(NAO_model, hard_spec, soft_spec)
###############################
############## static plan synthesis
start = time.time()
#STYLE = 'on-the-fly'
STYLE = 'static'
NAO_planner.optimal(10, STYLE)
print 'full construction and synthesis done within %.2fs \n' %(time.time()-start)


