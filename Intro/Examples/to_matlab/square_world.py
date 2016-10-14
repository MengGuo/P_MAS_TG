#!/usr/bin/python
# -*- coding: utf-8 -*-

# export PYTHONPATH=$PYTHONPATH:/to/your/P_MAS_TG

import time

from P_MAS_TG.ts import MotionFts, ActionModel, MotActModel
from P_MAS_TG.planner import ltl_planner


rectworld_nodename = "r{:d}c{:d}".format

def create_rectworld(rows, columns, eight_connected=False):
    '''
    create a rectangular world with square cells.
    eight_connected True if diagonal motion is allowed.
    '''
    node_dict = {}
    symbols = []
    row_range = range(0, rows)
    column_range = range(0, columns)
    for row in row_range:
        for col in column_range:
            node_name = rectworld_nodename(row, col)
            node_dict[(col,row)] = set([node_name,])
            symbols.append(node_name)
    g = MotionFts(node_dict, symbols, 'rectworld')
    for row in row_range:
        for col in column_range:
            nodef = (row, col)
            if eight_connected:
                offsets = product([-1,0,1],[-1,0,1])
            else:
                offsets = [(0,0),(0,1),(0,-1),(1,0),(-1,0)]
            for (dr,dc) in offsets:
                rt = row + dr
                ct = col + dc
                if rt not in row_range or ct not in column_range:
                    continue
                nodet = (rt, ct)
                unit_cost = 1
                g.add_edge(nodef, nodet, weight=unit_cost)
    return g


# create motion model
robot_motion = create_rectworld(4,4,False)
robot_motion.set_initial((0,0))

# empty action model
robot_action = ActionModel(dict())

# complete robot model
robot_model = MotActModel(robot_motion, robot_action)


# task formula
hard_task = '<> [] r3c3'
soft_task = None

# set planner
robot_planner = ltl_planner(robot_model, hard_task, soft_task)

# synthesis
start = time.time()
robot_planner.optimal(10,'static')

print 'full construction and synthesis done within %.2fs \n' %(time.time()-start)

#----------------------------------------
#----------------------------------------
# save Buchi automata to csv.dat that Matlab wants
# important to transform string names to indexs
buchi = robot_planner.product.graph['buchi']
nodes_list = buchi.nodes()
# save node name, index pairs
# also save initial, accept states
f_buchi_node = open('buchi_node.dat','w')
f_buchi_initial = open('buchi_node_initial.dat','w')
f_buchi_accept = open('buchi_node_accept.dat','w')
for nd_id, nd in enumerate(nodes_list):
    f_buchi_node.write('%d,%s\n' %(nd_id, nd))
    if nd in buchi.graph['initial']:
        f_buchi_initial.write('%d\n' %nd_id)
    if nd in buchi.graph['accept']:
        f_buchi_accept.write('%d\n' %nd_id)        
f_buchi_node.close()
f_buchi_initial.close()
f_buchi_accept.close()
# save edges, node name swapped by index
f_buchi_edge = open('buchi_edge.dat','w')
for (ef,et) in buchi.edges_iter():
    id_ef = nodes_list.index(ef)
    id_et = nodes_list.index(et)
    f_buchi_edge.write('%d,%d\n' %(id_ef, id_et)) 
f_buchi_edge.close()


#----------------------------------------
#----------------------------------------
# save Prod automata to csv.dat that Matlab wants
# important to transform string names to indexs
prod = robot_planner.product
prod_nodes_list = prod.nodes()
# save node name, index pairs
# also save initial, accept states
f_prod_node = open('prod_node.dat','w')
f_prod_initial = open('prod_node_initial.dat','w')
f_prod_accept = open('prod_node_accept.dat','w')
for nd_id, nd in enumerate(prod_nodes_list):
    f_prod_node.write('%d,%s\n' %(nd_id, nd))
    if nd in prod.graph['initial']:
        f_prod_initial.write('%d\n' %nd_id)
    if nd in prod.graph['accept']:
        f_prod_accept.write('%d\n' %nd_id)        
f_prod_node.close()
f_prod_initial.close()
f_prod_accept.close()
# save edges, node name swapped by index
f_prod_edge = open('prod_edge.dat','w')
for (ef,et) in prod.edges_iter():
    id_ef = prod_nodes_list.index(ef)
    id_et = prod_nodes_list.index(et)
    f_prod_edge.write('%d,%d\n' %(id_ef, id_et)) 
f_prod_edge.close()



#-------------------
# load all .dat by 'csvread()' in matlab
#-------------------
