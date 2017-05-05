# -*- coding: utf-8 -*-

from buchi import check_label_for_buchi_edge

from networkx.classes.digraph import DiGraph


class ProdAut(DiGraph):
	def __init__(self, ts, buchi, alpha=100):
		DiGraph.__init__(self, ts=ts, buchi=buchi, alpha=alpha, initial=set(), accept=set(), type='ProdAut')

	def build_full(self):
		for f_ts_node in self.graph['ts'].nodes_iter():
			for f_buchi_node in self.graph['buchi'].nodes_iter():
				f_prod_node = self.composition(f_ts_node, f_buchi_node)
                                #print 'f_prod_node' , (f_ts_node, f_buchi_node)
				for t_ts_node in self.graph['ts'].successors_iter(f_ts_node):
					for t_buchi_node in self.graph['buchi'].successors_iter(f_buchi_node):
							t_prod_node = self.composition(t_ts_node, t_buchi_node)
                                                        #print 't_prod_node' , (t_ts_node, t_buchi_node)
							label = self.graph['ts'].node[f_ts_node]['label']
							cost = self.graph['ts'][f_ts_node][t_ts_node]['weight']
							truth, dist = check_label_for_buchi_edge(self.graph['buchi'], label, f_buchi_node, t_buchi_node)
							total_weight = cost + self.graph['alpha']*dist
                                                        #print 'label,truth,total_weight', label,truth,total_weight
							if truth:
								self.add_edge(f_prod_node, t_prod_node, weight=total_weight)
                                                                #print 'add edge', (f_prod_node, t_prod_node)
                print 'full product constructed with %d states and %s transitions' %(len(self.nodes()), len(self.edges()))                                                                 

	def composition(self, ts_node, buchi_node):
		prod_node = (ts_node, buchi_node)
		if not self.has_node(prod_node):
			self.add_node(prod_node, ts=ts_node, buchi=buchi_node, marker='unvisited')
			if ((ts_node in self.graph['ts'].graph['initial']) and
				(buchi_node in self.graph['buchi'].graph['initial'])):
				self.graph['initial'].add(prod_node)
			if (buchi_node in self.graph['buchi'].graph['accept']):
				self.graph['accept'].add(prod_node)
		return prod_node

	def projection(self, prod_node):
		ts_node = self.node[prod_node]['ts']
		buchi_node = self.node[prod_node]['buchi']
		return ts_node, buchi_node

	def build_initial(self):
		self.graph['ts'].build_initial()
		for ts_init in self.graph['ts'].graph['initial']:
			for buchi_init in self.graph['buchi'].graph['initial']:
				init_prod_node = self.composition(ts_init, buchi_init)

	def build_accept(self):
		self.graph['ts'].build_full()
		accept = set()
		for ts_node in self.graph['ts'].nodes_iter():
			for buchi_accept in self.graph['buchi'].graph['accept']:
				accept_prod_node = self.composition(ts_node, buchi_accept)

	def accept_predecessors(self, accept_node):
		pre_set = set()
		t_ts_node, t_buchi_node = self.projection(accept_node)
		for f_ts_node, cost in self.graph['ts'].fly_predecessors_iter(t_ts_node):
			for f_buchi_node in self.graph['buchi'].predecessors_iter(t_buchi_node):
				f_prod_node = self.composition(f_ts_node, f_buchi_node)
				label = self.graph['ts'].node[f_ts_node]['label']
				truth, dist = check_label_for_buchi_edge(self.graph['buchi'], label, f_buchi_node, t_buchi_node)
				total_weight = cost + self.graph['alpha']*dist
				if truth:
					pre_set.add(f_prod_node)
					self.add_edge(f_prod_node, accept_node, weight=total_weight)
		return pre_set

	def fly_successors_iter(self, f_prod_node):
		f_ts_node, f_buchi_node = self.projection(f_prod_node)
		# been visited before, and hasn't changed 
		if ((self.node[f_prod_node]['marker'] == 'visited') and 
			(self.graph['ts'].graph['region'].node[
				self.graph['ts'].node[self.node[f_prod_node]['ts']]['region']]['status'] == 'confirmed')):
			for t_prod_node in self.successors_iter(f_prod_node):
				yield t_prod_node, self.edge[f_prod_node][t_prod_node]['weight']
		else:
			self.remove_edges_from(self.out_edges(f_prod_node))
			for t_ts_node,cost in self.graph['ts'].fly_successors_iter(f_ts_node):
				for t_buchi_node in self.graph['buchi'].successors_iter(f_buchi_node):
					t_prod_node = self.composition(t_ts_node, t_buchi_node)
					label = self.graph['ts'].node[f_ts_node]['label']
					truth, dist = check_label_for_buchi_edge(self.graph['buchi'], label, f_buchi_node, t_buchi_node)
					total_weight = cost + self.graph['alpha']*dist
					if truth:
						self.add_edge(f_prod_node, t_prod_node, weight=total_weight)
						yield t_prod_node, total_weight
			self.node[f_prod_node]['marker'] = 'visited'


class ProdAut_Run(object):
	# prefix, suffix in product run
	# prefix: init --> accept, suffix accept --> accept
	# line, loop in ts
	def __init__(self, product, prefix, precost, suffix, sufcost, totalcost):
		self.prefix = prefix
		self.precost = precost
		self.suffix = suffix
		self.sufcost = sufcost
		self.totalcost = totalcost
		#self.prod_run_to_prod_edges(product)
		self.plan_output(product)
		#self.plan = chain(self.line, cycle(self.loop))
		#self.plan = chain(self.loop)

	def prod_run_to_prod_edges(self, product):
		self.pre_prod_edges = zip(self.prefix[0:-2], self.prefix[1:-1])
		self.suf_prod_edges = zip(self.suffix[0:-2], self.suffix[1:-1])

	def plan_output(self, product):
		self.line = [product.node[node]['ts'] for node in self.prefix]
		self.loop = [product.node[node]['ts'] for node in self.suffix]
                if len(self.line) == 2:
                        self.pre_ts_edges = [(self.line[0], self.line[1])]
                else:
		        self.pre_ts_edges = zip(self.line[0:-1], self.line[1:])
                if len(self.loop) == 2:
                        self.suf_ts_edges = [(self.loop[0], self.loop[1])]
                else:
		        self.suf_ts_edges = zip(self.loop[0:-1], self.loop[1:])
                self.suf_ts_edges.append((self.loop[-1],self.loop[0]))
		# output plan
		self.pre_plan = []
		self.pre_plan.append(self.line[0][0]) 
		for ts_edge in self.pre_ts_edges:
			if product.graph['ts'][ts_edge[0]][ts_edge[1]]['label'] == 'goto':
				self.pre_plan.append(ts_edge[1][0]) # motion 
			else:
				self.pre_plan.append(ts_edge[1][1]) # action
                bridge = (self.line[-1],self.loop[0])
                if product.graph['ts'][bridge[0]][bridge[1]]['label'] == 'goto':
			self.pre_plan.append(bridge[1][0]) # motion 
		else:
			self.pre_plan.append(bridge[1][1]) # action
		self.suf_plan = []		
		for ts_edge in self.suf_ts_edges:
			if product.graph['ts'][ts_edge[0]][ts_edge[1]]['label'] == 'goto':
				self.suf_plan.append(ts_edge[1][0]) # motion 
			else:
				self.suf_plan.append(ts_edge[1][1]) # action










