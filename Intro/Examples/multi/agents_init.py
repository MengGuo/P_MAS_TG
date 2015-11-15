from P_MAS_TG.ts import MotionFts, ActionModel, MotActModel
from P_MAS_TG.planner import ltl_planner



#=================================
dep = dict()
SPEED = [1, 0.8, 0.6, 0.4, 0.5, 0.7,]
COST = 10
LOCAL  = {'la':COST,'ua':COST, 's':COST, 'md':COST, 'oe':COST,'None':1}
#=================================
# A1 fts
A1_init_pose = [2.0,2.0,0.0]
A1_node_dict = {
 (2.0,2.0,0.1): set(['r0']),
 (0.5,0.5,0.2): set(['r1','a']),
 (3.5,0.5,0.2): set(['r2']),
 (0.5,2.0,0.2): set(['r3']),
 (3.5,2.0,0.2): set(['r4','b']),
 (0.5,3.3,0.2): set(['r5']),
 (3.5,3.3,0.2): set(['r6']),
 (2.0,1.0,0.3): set(['r7']),
 (2.0,3.0,0.3): set(['r8']),
}
A1_symbols = set(['r0','r1','r2','r3','r4','r5','r6','r7','r8','a','b'])
A1_motion = MotionFts(A1_node_dict, A1_symbols, 'A1_FTS')
A1_motion.set_initial(A1_init_pose)
A1_motion.add_full_edges(unit_cost = 1.0/SPEED[0])
# A1 action
A1_action_dict= {'la': (COST,'a',set([ 'la'])),
				'ua': (COST,'1',set([ 'ua'])),
				'lb': (COST,'b',set([ 'lb'])),
				'ub': (COST,'1',set([ 'ub'])),}
dep['lb']=set(['hb',])
dep['ub']=set(['hb',])
A1_action = ActionModel(A1_action_dict)
# A1 task
A1_hard_task = '<>(la && <>(ua && r2)) && <>(lb && <>(ub && r3)) && ([]<>r0)'
#A1_hard_task = '<> t1'
A1_soft_task = None
# A1 model, planner 
A1_model = MotActModel(A1_motion, A1_action)
A1_planner = ltl_planner(A1_model, A1_hard_task, A1_soft_task)

#=================================
# A2 fts
A2_init_pose = [2.0,2.0,0.0]
A2_node_dict = {
 (2.0,2.0,0.1): set(['r0']),
 (0.5,0.5,0.2): set(['r1']),
 (3.5,0.5,0.2): set(['r2']),
 (0.5,2.0,0.2): set(['r3']),
 (3.5,2.0,0.2): set(['r4']),
 (0.5,3.3,0.2): set(['r5']),
 (3.5,3.3,0.2): set(['r6']),
 (2.0,1.0,0.3): set(['r7']),
 (2.0,3.0,0.3): set(['r8']),
}
A2_symbols = set(['r0','r1','r2','r3','r4','r5','r6','r7','r8'])
A2_motion = MotionFts(A2_node_dict, A2_symbols, 'A2_FTS')
A2_motion.set_initial(A2_init_pose)
A2_motion.add_full_edges(unit_cost = 1.0/SPEED[1])
# A2 action
A2_action_dict= {'s': (COST,'1',set(['s'])),
				'hb': (COST,'1',set()),
				'hc1': (COST,'1',set()),
				'hf': (COST,'1',set()),}
A2_action = ActionModel(A2_action_dict)
# A2 task
A2_hard_task = '<>((s && r7)) && <>((s && r8))&& ([]<>r0)'
#A2_hard_task = '<> t1'
A2_soft_task = None
# A2 model, planner 
A2_model = MotActModel(A2_motion, A2_action)
A2_planner = ltl_planner(A2_model, A2_hard_task, A2_soft_task)

#=================================
# A3 fts
A3_init_pose = [2.0,2.0,0.0]
A3_node_dict = {
 (2.0,2.0,0.1): set(['r0']),
 (0.5,0.5,0.2): set(['r1']),
 (3.5,0.5,0.2): set(['r2']),
 (0.5,2.0,0.2): set(['r3']),
 (3.5,2.0,0.2): set(['r4']),
 (0.5,3.3,0.2): set(['r5']),
 (3.5,3.3,0.2): set(['r6']),
 (2.0,1.0,0.3): set(['r7']),
 (2.0,3.0,0.3): set(['r8','m']),
}
A3_symbols = set(['r0','r1','r2','r3','r4','r5','r6','r7','r8','m'])
A3_motion = MotionFts(A3_node_dict, A3_symbols, 'A3_FTS')
A3_motion.set_initial(A3_init_pose)
A3_motion.add_full_edges(unit_cost = 1.0/SPEED[2])
# A3 action
A3_action_dict= {'om': (COST,'m',set(['om'])),
				'hc2': (COST,'1',set()),
				'hf': (COST,'1',set()),}
dep['om']=set(['hm',])
A3_action = ActionModel(A3_action_dict)
# A3 task
A3_hard_task = '<>(om && <>r6)&& ([]<>r0)'
#A3_hard_task = '<> t1'
A3_soft_task = None
# A3 model, planner 
A3_model = MotActModel(A3_motion, A3_action)
A3_planner = ltl_planner(A3_model, A3_hard_task, A3_soft_task)

#=================================
# A4 fts
A4_init_pose = [2.0,2.0,0.0]
A4_node_dict = {
 (2.0,2.0,0.1): set(['r0']),
 (0.5,0.5,0.2): set(['r1']),
 (3.5,0.5,0.2): set(['r2']),
 (0.5,2.0,0.2): set(['r3']),
 (3.5,2.0,0.2): set(['r4']),
 (0.5,3.3,0.2): set(['r5','c']),
 (3.5,3.3,0.2): set(['r6']),
 (2.0,1.0,0.3): set(['r7']),
 (2.0,3.0,0.3): set(['r8',]),
}
A4_symbols = set(['r0','r1','r2','r3','r4','r5','r6','r7','r8','c'])
A4_motion = MotionFts(A4_node_dict, A4_symbols, 'A4_FTS')
A4_motion.set_initial(A4_init_pose)
A4_motion.add_full_edges(unit_cost = 1.0/SPEED[3])
# A4 action
A4_action_dict= {'s': (COST,'1',set(['s'])),
				'ac': (COST,'c',set(['ac'])),
				'hf': (COST,'1',set()),
				'hm': (COST,'1',set()),}
dep['ac']=set(['hc1','hc2',])
A4_action = ActionModel(A4_action_dict)
# A4 task
A4_hard_task = '<>((s && r7)) && <>((ac && <>r0))&& ([]<>r0)'
#A4_hard_task = '<> t1'
A4_soft_task = None
# A4 model, planner 
A4_model = MotActModel(A4_motion, A4_action)
A4_planner = ltl_planner(A4_model, A4_hard_task, A4_soft_task)

#=================================
# A5 fts
A5_init_pose = [2.0,2.0,0.0]
A5_node_dict = {
 (2.0,2.0,0.1): set(['r0']),
 (0.5,0.5,0.2): set(['r1']),
 (3.5,0.5,0.2): set(['r2']),
 (0.5,2.0,0.2): set(['r3']),
 (3.5,2.0,0.2): set(['r4']),
 (0.5,3.3,0.2): set(['r5']),
 (3.5,3.3,0.2): set(['r6']),
 (2.0,1.0,0.3): set(['r7','d']),
 (2.0,3.0,0.3): set(['r8',]),
}
A5_symbols = set(['r0','r1','r2','r3','r4','r5','r6','r7','r8','d'])
A5_motion = MotionFts(A5_node_dict, A5_symbols, 'A5_FTS')
A5_motion.set_initial(A5_init_pose)
A5_motion.add_full_edges(unit_cost = 1.0/SPEED[4])
# A5 action
A5_action_dict= {'md': (COST,'d',set(['md'])),
				'hb': (COST,'1',set()),
				'hc1': (COST,'1',set()),
				'hc2': (COST,'1',set()),}
A5_action = ActionModel(A5_action_dict)
# A5 task
A5_hard_task = '<>((md && <>r0))&& ([]<>r0)'
#A5_hard_task = '<> t1'
A5_soft_task = None
# A5 model, planner 
A5_model = MotActModel(A5_motion, A5_action)
A5_planner = ltl_planner(A5_model, A5_hard_task, A5_soft_task)

#=================================
# A6 fts
A6_init_pose = [2.0,2.0,0.0]
A6_node_dict = {
 (2.0,2.0,0.1): set(['r0']),
 (0.5,0.5,0.2): set(['r1','e']),
 (3.5,0.5,0.2): set(['r2']),
 (0.5,2.0,0.2): set(['r3','f']),
 (3.5,2.0,0.2): set(['r4']),
 (0.5,3.3,0.2): set(['r5']),
 (3.5,3.3,0.2): set(['r6']),
 (2.0,1.0,0.3): set(['r7',]),
 (2.0,3.0,0.3): set(['r8',]),
}
A6_symbols = set(['r0','r1','r2','r3','r4','r5','r6','r7','r8','e','f'])
A6_motion = MotionFts(A6_node_dict, A6_symbols, 'A6_FTS')
A6_motion.set_initial(A6_init_pose)
A6_motion.add_full_edges(unit_cost = 1.0/SPEED[5])
# A6 action
A6_action_dict= {'oe': (COST,'e',set(['oe'])),
				'cf': (COST,'f',set(['cf'])),
				'hb': (COST,'1',set()),
				'hm': (COST,'1',set()),}
dep['cf']=set(['hf',])					
A6_action = ActionModel(A6_action_dict)
# A6 task
A6_hard_task = '<>((oe && <> (cf && <>r0 )))&& ([]<>r0)'
#A6_hard_task = '<> t1'
A6_soft_task = None
# A6 model, planner 
A6_model = MotActModel(A6_motion, A6_action)
A6_planner = ltl_planner(A6_model, A6_hard_task, A6_soft_task)


initial_pose= (list(A1_init_pose),list(A2_init_pose),list(A3_init_pose),
	list(A4_init_pose), list(A5_init_pose), list(A6_init_pose),)

Region = dict()
Region['r0'] = (2.0,2.0,0.1)
Region['r1'] = (0.5,0.5,0.2)
Region['r2'] = (3.5,0.5,0.2)
Region['r3'] = (0.5,2.0,0.2)
Region['r4'] = (3.5,2.0,0.2)
Region['r5'] = (0.5,3.3,0.2)
Region['r6'] = (3.5,3.3,0.2)
Region['r7'] = (2.0,1.0,0.3)
Region['r8'] = (2.0,3.0,0.3)