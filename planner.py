# -*- coding: utf-8 -*-
from buchi import mission_to_buchi
from product import ProdAut
from ts import distance, reach_waypoint
from discrete_plan import dijkstra_plan_networkX, dijkstra_plan_optimal
from graphics import visualize_office


class ltl_planner(object):
	def __init__(self, ts, hard_spec, soft_spec):
		buchi = mission_to_buchi(hard_spec, soft_spec)
		self.product = ProdAut(ts, buchi)
		self.Time = 0
		self.cur_pose = None
		self.trace = [] # record the regions been visited
		self.traj = [] # record the full trajectory
		self.opt_log = [] 
		# record [(time, prefix, suffix, prefix_cost, suffix_cost, total_cost)]
		self.com_log = []
		# record [(time, no_messages)]

	def optimal(self, beta=10, style='static'):
		self.beta = beta
		if style == 'static':
			# full graph construction
			self.product.graph['ts'].build_full()
			self.product.build_full()
			self.run, plantime = dijkstra_plan_networkX(self.product, self.beta)
		elif style == 'ready':
			self.product.build_full()
			self.run, plantime = dijkstra_plan_networkX(self.product, self.beta)
		elif style == 'on-the-fly':
			# on-the-fly construction
			self.product.build_initial()
			self.product.build_accept()
			self.run, plantime = dijkstra_plan_optimal(self.product, self.beta)
		print 'the plan prefix:\n'
		#print [[n, self.product.graph['ts'].graph['region'].node[n]['label']] for n in self.run.pre_plan]
		print [n for n in self.run.pre_plan]
		print '\n'
		print 'the plan suffix:\n'
		#print [[n, self.product.graph['ts'].graph['region'].node[n]['label']] for n in self.run.suf_plan]
		print [n for n in self.run.suf_plan]
		print '\n'
		self.opt_log.append((self.Time, self.run.pre_plan, self.run.suf_plan, self.run.precost, self.run.sufcost, self.run.totalcost))
		self.last_time = self.Time
		self.acc_change = 0
		self.index = 0
		self.segment = 'line'
		self.next_move = self.run.pre_plan[self.index]
		return plantime

	def find_next_move(self):
		if self.segment == 'line' and self.index < len(self.run.pre_plan)-1:
				self.index += 1
				self.next_move = self.run.pre_plan[self.index]
		elif self.segment == 'line' and self.index == len(self.run.pre_plan)-1:
				self.index = 0
				self.segment = 'loop'
				self.next_move = self.run.suf_plan[self.index]
		elif self.segment == 'loop' and self.index < len(self.run.suf_plan)-1:
				self.index += 1
				self.next_move = self.run.suf_plan[self.index]
		elif self.segment == 'loop' and self.index == len(self.run.suf_plan)-1:
				self.index = 0
				self.segment = 'loop'
				self.next_move = self.run.suf_plan[self.index]

	def off_line_execute(self, init_pose, track_error, motion_func, N=100):
		self.track_error = track_error
		self.cur_pose = init_pose
		while (len(self.trace)<N):
			if not self.next_move:
				self.optimal()
			self.trace.append(self.next_move)
			while not reach_waypoint(self.cur_pose, self.next_move, self.track_error):
				self.cur_pose = motion_func(self.cur_pose, self.next_move)
				self.traj.append(self.cur_pose)
			self.find_next_move()


	def output(self, filename, colormap):
	    plot_traj(filename, self.product.graph['ts'].graph['region'], self.traj, colormap)








