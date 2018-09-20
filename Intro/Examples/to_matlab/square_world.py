#!/usr/bin/python
# -*- coding: utf-8 -*-

# export PYTHONPATH=$PYTHONPATH:/to/your/P_MAS_TG

import time

from P_MAS_TG.ts import MotionFts, ActionModel, MotActModel
from P_MAS_TG.planner import ltl_planner
from networkx import draw_networkx, spring_layout
from itertools import product
import matplotlib.pyplot as plt


rectworld_nodename = "x{:d}y{:d}".format

def create_rectworld(Xs, Ys, eight_connected=False):
    '''
    create a rectangular world with square cells.
    eight_connected True if diagonal motion is allowed.
    '''
    node_dict = {}
    symbols = []
    X_range = range(0, Xs)
    Y_range = range(0, Ys)
    for X in X_range:
        for Y in Y_range:
            node_name = rectworld_nodename(X, Y)
            node_dict[(X,Y)] = set([node_name,])
            symbols.append(node_name)
    g = MotionFts(node_dict, symbols, 'rectworld')
    for X in X_range:
        for Y in Y_range:
            nodef = (X, Y)
            if eight_connected:
                offsets = product([-1,0,1],[-1,0,1])
            else:
                offsets = [(0,0),(0,1),(0,-1),(1,0),(-1,0)]
            for (dx,dy) in offsets:
                xt = X + dx
                yt = Y + dy
                if xt not in X_range or yt not in Y_range:
                    continue
                nodet = (xt, yt)
                unit_cost = 1
                g.add_edge(nodef, nodet, weight=unit_cost*(abs(dx)+abs(dy)))
    return g


# create motion model
robot_motion = create_rectworld(4,4,True)
robot_motion.set_initial((0,0))

# empty action model
robot_action = ActionModel(dict())

# complete robot model
robot_model = MotActModel(robot_motion, robot_action)


# task formula
hard_task = '(<> x0y3) && (<> x3y2)'
soft_task = None

# set planner
robot_planner = ltl_planner(robot_model, hard_task, soft_task)


# synthesis
start = time.time()
robot_planner.optimal(10,'static')

print '------------------------------'
print 'Full construction and synthesis done within %.2fs' %(time.time()-start)



#----------------------------------------
#----------------------------------------
# save transition system to csv.dat that Matlab wants
# important to transform string names to indexs
ts = robot_planner.product.graph['ts']
ts_nodes_list = ts.nodes()
# save node name, index pairs
# also save initial, accept stats
f_ts_node = open('data/ts_node.dat','w')
f_ts_initial = open('data/ts_node_initial.dat','w')
for nd_id, nd in enumerate(ts_nodes_list):
    # ts_node_id, ts_node_x, ts_node_y
    f_ts_node.write('%d,%d,%d\n' %(nd_id, nd[0][0], nd[0][1]))
    ts.nodes[nd]['index'] = nd_id
    if nd in ts.graph['initial']:
        f_ts_initial.write('%d\n' %nd_id)
f_ts_node.close()
f_ts_initial.close()
# save edges, node name swapped by index
f_ts_edge = open('data/ts_edge.dat','w')
for e in ts.edges():
    id_ef = ts.nodes[e[0]]['index']
    id_et = ts.nodes[e[1]]['index']
    f_ts_edge.write('%d,%d\n' %(id_ef, id_et)) 
f_ts_edge.close()

#----------------------------------------
#----------------------------------------
# save Buchi automata to csv.dat that Matlab wants
# important to transform string names to indexs
buchi = robot_planner.product.graph['buchi']
buchi_nodes_list = buchi.nodes()
# save node name, index pairs
# also save initial, accept states
f_buchi_node = open('data/buchi_node.dat','w')
f_buchi_initial = open('data/buchi_node_initial.dat','w')
f_buchi_accept = open('data/buchi_node_accept.dat','w')
for nd_id, nd in enumerate(buchi_nodes_list):
    buchi.nodes[nd]['index'] = nd_id
    f_buchi_node.write('%d\n' %nd_id)
    if nd in buchi.graph['initial']:
        f_buchi_initial.write('%d\n' %nd_id)
    if nd in buchi.graph['accept']:
        f_buchi_accept.write('%d\n' %nd_id)        
f_buchi_node.close()
f_buchi_initial.close()
f_buchi_accept.close()
# save edges, node name swapped by index
f_buchi_edge = open('data/buchi_edge.dat','w')
for e in buchi.edges():
    id_ef = buchi.nodes[e[0]]['index']
    id_et = buchi.nodes[e[1]]['index']
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
f_prod_node = open('data/prod_node.dat','w')
f_prod_initial = open('data/prod_node_initial.dat','w')
f_prod_accept = open('data/prod_node_accept.dat','w')
for nd_id, nd in enumerate(prod_nodes_list):
    #f_prod_node.write('%d,%s\n' %(nd_id, nd))
    # prod_node_id, ts_node_x, ts_node_y
    prod.nodes[nd]['index'] = nd_id
    f_prod_node.write('%d,%d,%d,%d\n' %(nd_id, nd[0][0][0], nd[0][0][1], buchi.nodes[nd[1]]['index']))
    if nd in prod.graph['initial']:
        f_prod_initial.write('%d\n' %nd_id)
    if nd in prod.graph['accept']:
        f_prod_accept.write('%d\n' %nd_id)        
f_prod_node.close()
f_prod_initial.close()
f_prod_accept.close()
# save edges, node name swapped by index
f_prod_edge = open('data/prod_edge.dat','w')
for e in prod.edges():
    id_ef = prod.nodes[e[0]]['index']
    id_et = prod.nodes[e[1]]['index']
    f_prod_edge.write('%d,%d\n' %(id_ef, id_et)) 
f_prod_edge.close()


print '------------------------------'
print 'Check *.mat files and load them in Matlab.'



print '------------------------------'
print 'Check *.pdf for visualization of ts, buchi and prod'
draw_networkx(ts,pos=spring_layout(ts))
plt.savefig('figures/ts.pdf',bbox_inches='tight')
plt.clf()

draw_networkx(buchi,pos=spring_layout(buchi))
plt.savefig('figures/buchi.pdf',bbox_inches='tight')
plt.clf()

draw_networkx(prod,pos=spring_layout(prod))
plt.savefig('figures/prod.pdf',bbox_inches='tight')
#-------------------
# load all .dat by 'csvread()' in matlab
#-------------------