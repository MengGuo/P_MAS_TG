# -*- coding: utf-8 -*-

from ltl2ba import run_ltl2ba
from promela import parse as parse_ltl, find_states, find_symbols
from boolean_formulas.parser import parse as parse_guard
from itertools import product as cartesian_product

from networkx.classes.digraph import DiGraph

def buchi_from_ltl(formula,Type):
    promela_string = run_ltl2ba(formula)
    symbols = find_symbols(formula)
    edges = parse_ltl(promela_string)
    (states, initials, accepts) = find_states(edges)
    buchi = DiGraph(type=Type, initial=initials, accept=accepts, symbols=symbols)
    for state in states:
        buchi.add_node(state)
    for (ef,et) in edges.keys():
        guard_formula = edges[(ef,et)]
        guard_expr = parse_guard(guard_formula)
        buchi.add_edge(ef, et, guard=guard_expr, guard_formula=guard_formula)
    return buchi

def mission_to_buchi(hard_spec, soft_spec):
    if (hard_spec and not soft_spec):
        buchi = buchi_from_ltl(hard_spec,'hard_buchi')
    elif (soft_spec and not hard_spec):
        buchi = buchi_from_ltl(soft_spec,'soft_buchi')
    elif (hard_spec and soft_spec):
        buchi = DuoBA_from_ltls(hard_spec, soft_spec)
    print 'full Buchi constructed with %d states and %s transitions' %(len(buchi.nodes()), len(buchi.edges()))         
    return buchi

def DuoBA_from_ltls(hard_spec, soft_spec):
    hard_buchi = buchi_from_ltl(hard_spec, 'hard_buchi')
    soft_buchi = buchi_from_ltl(soft_spec, 'soft_buchi')
    hard_symbols = hard_buchi.graph['symbols']
    soft_symbols = soft_buchi.graph['symbols']
    symbols = set(hard_symbols).union(set(soft_symbols))
    DuoBA = DiGraph(type='safe_buchi', hard=hard_buchi, soft=soft_buchi, symols=symbols)
    initial = set()
    accept = set()
    for (h_node, s_node, l) in cartesian_product(hard_buchi.nodes(), soft_buchi.nodes(), [1, 2]):
        DuoNode = (h_node, s_node, l)
        DuoBA.add_node(DuoNode,hard=h_node, soft=s_node, level=l)
        if (h_node in hard_buchi.graph['initial'] and 
            s_node in soft_buchi.graph['initial'] and l == 1):
            initial.add(DuoNode)
        if (h_node in hard_buchi.graph['accept'] and l == 1):
            accept.add(DuoNode)
    DuoBA.graph['accept'] = accept
    DuoBA.graph['initial'] = initial
    for f_duonode in DuoBA.nodes_iter():
        for t_duonode in DuoBA.nodes_iter():
            f_h_node, f_s_node, f_level = check_duo_attr(DuoBA, f_duonode)
            t_h_node, t_s_node, t_level = check_duo_attr(DuoBA, t_duonode)
            if (t_h_node not in DuoBA.graph['hard'].neighbors(f_h_node) or 
                t_s_node not in DuoBA.graph['soft'].neighbors(f_s_node)):
                continue
                # relaxed because no common input alphabets are enabled
            hardguard = DuoBA.graph['hard'].edge[f_h_node][t_h_node]['guard']
            softguard = DuoBA.graph['soft'].edge[f_s_node][t_s_node]['guard']
            if ((f_h_node not in DuoBA.graph['hard'].graph['accept'] and 
                f_level == 1 and t_level == 1) or 
                (f_h_node in DuoBA.graph['hard'].graph['accept'] and 
                f_level == 1 and t_level == 2) or 
                (f_s_node not in DuoBA.graph['soft'].graph['accept'] and 
                f_level == 2 and t_level == 2) or 
                (f_s_node in DuoBA.graph['soft'].graph['accept'] and 
                f_level == 2 and t_level == 1)):
                DuoBA.add_edge(f_duonode, t_duonode, hardguard=hardguard, softguard=softguard)
    return DuoBA

def check_duo_attr(DuoBA, node):
    return DuoBA.node[node]['hard'], DuoBA.node[node]['soft'], DuoBA.node[node]['level']

def check_label_for_buchi_edge(buchi, label, f_buchi_node, t_buchi_node):
    buchi_type = buchi.graph['type']
    if buchi_type == 'hard_buchi':
        truth = buchi.edge[f_buchi_node][t_buchi_node]['guard'].check(label)
        dist = 0
    if buchi_type == 'soft_buchi':
        truth = True
        dist = buchi.edge[f_buchi_node][t_buchi_node]['guard'].distance(label)
    if buchi_type == 'safe_buchi':
        truth = buchi.edge[f_buchi_node][t_buchi_node]['hardguard'].check(label)
        if truth:
            dist = buchi.edge[f_buchi_node][t_buchi_node]['softguard'].distance(label)
        else:
            dist = 1000
    return truth, dist
