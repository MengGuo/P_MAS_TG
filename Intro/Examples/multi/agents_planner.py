from math import sqrt
from matplotlib import pyplot
import matplotlib.patches
from matplotlib.backends.backend_pdf import PdfPages
from collections import defaultdict
import random

from P_MAS_TG.motion import unicycle_turn_forward

from init import *

def distance(pose1, pose2):
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)			
				
def reach(x, reg):
	if ((abs(x[0]-reg[0])<= reg[2]*0.6) 
		and (abs(x[1]-reg[1])<= reg[2]*0.6)):
		return True
	return False

def movie_clips(Region, XX=None, L=10):
    K = range(0, len(XX[0]), L)
    for k in xrange(1,len(K)):
    	X1 = [XX[0][n] for n in K[0:(k+1)]]
    	X2 = [XX[1][n] for n in K[0:(k+1)]]
    	X3 = [XX[2][n] for n in K[0:(k+1)]]
    	X4 = [XX[3][n] for n in K[0:(k+1)]]
    	X5 = [XX[4][n] for n in K[0:(k+1)]]
    	X6 = [XX[5][n] for n in K[0:(k+1)]]
    	XX_hat = [X1,X2,X3,X4,X5,X6]
        figure = visualize_traj(Region, XX_hat)
        figure.savefig('movie/frame%s.png' %k)
        #save_pdf('movie/frame%s' %k, figure)


def visualize_traj(Region, XX=None, L=1, Ed=None):
    # [(time, no_messages)]
    figure = pyplot.figure()
    axes = figure.add_subplot(1,1,1)
    marker = ['o', 's', '^', '*', 'v','>']
    color = ['r', 'g', 'b', 'c','m','y']
    for name, reg in Region.iteritems():
        rect = matplotlib.patches.Rectangle(
            [reg[0]-reg[2],reg[1]-reg[2]],
            2*reg[2], 2*reg[2],
            facecolor='gray',
            #fill = False,
            edgecolor='black',
            linewidth=1,
            ls='solid',
            alpha=0.7)
        axes.add_patch(rect)
        axes.text(reg[0]-0.2*reg[2],reg[1]-0.2*reg[2],r'%s' %name,
        	fontsize=12, fontweight ='bold')
    ########### plot traj
    if XX:
    	for i in xrange(0,6):
    		if Ed:
	        	xd = [n[0]   for n in XX[i][0:Ed:L]]
	        	yd = [n[1]  for n in  XX[i][0:Ed:L]]
	        else:
	        	xd = [n[0]   for n in XX[i][0::L]]
	        	yd = [n[1]  for n in  XX[i][0::L]]
	        line = matplotlib.lines.Line2D(
	            xd, yd, 
	            marker=marker[i], 
	            markersize=7,
	            markerfacecolor=color[i],
	            linestyle='-',
	            linewidth=2.5,
	            color=color[i],
	            label = 'Robot %s' %str(i+1),
	            alpha=1)
	        axes.add_line(line)
    axes.set_aspect('equal')
    axes.set_xlim(0, 4)
    axes.set_ylim(0, 4)
    #axes.set_axis_off()
    axes.set_xlabel('y (m)')
    axes.set_ylabel('x (m)')
    axes.legend(ncol=3, bbox_to_anchor=(0., 0.89, 1., .102), loc=3, 
    	mode="expand", borderaxespad=0,shadow=True)
    axes.grid("on") 
    #pyplot.savefig('%s.pdf' %name,bbox_inches='tight')
    return figure

def save_pdf(filename, fig):
    pp = PdfPages('%s.pdf' %filename)
    pp.savefig(fig)
    pp.close()
###################################################
#####################################################

############################################
STEP = 0.1
t=0
tt=0
f = [0,]*6 # for recording plan progress
K=[0,]*6 # for counting down action time
T=[0,]*6
horizon = [20,]*6
ACT = ['action',]*6
DELAY = 3
Request = dict()
Reply = dict()
T_bound = 500
###################
# initialize 
Planner = dict()
Planner[0] = A1_planner
Planner[1] = A2_planner
Planner[2] = A3_planner
Planner[3] = A4_planner
Planner[4] = A5_planner
Planner[5] = A6_planner
X=list(initial_pose)
# initial plan synthesis
for i in xrange(0,6):
    print 'Agent A%s' %(i+1)
    Planner[i].optimal(segment='prefix')
    #Planner[i].optimal()
while(tt<T_bound):
################
#while(True):
	Request.clear()
	Reply.clear()
	print 'Time %s:' %tt
	for i in xrange(0,6):
		Planner[i].pose = X[i]
		Planner[i].traj.append(list(X[i]))
		todo = Planner[i].next_move
		#======================
		if not isinstance(todo, basestring):
			# motions 
			reg = todo[:]
			if not reach(X[i], reg):
				##### motion 
				u1 = (SPEED[i]*(reg[0]-X[i][0])/distance(reg, X[i]) 
						+ STEP*random.gauss(0,2*STEP)  )
				u2 = (SPEED[i]*(reg[1]-X[i][1])/distance(reg, X[i])
						+ STEP*random.gauss(0,2*STEP)  )
				X[i][0] += u1*STEP
				X[i][1] += u2*STEP
				#print ('agent%s now doing local motion to %s\n' 
				#				%(str(i+1), str(reg)))
				#X[i] = unicycle_turn_forward(X[i], reg,
						#time_step=STEP, x_speed=SPEED[i],turn_speed=1)
				if Planner[i].contract_time>0:
					Planner[i].contract_time -= STEP
			else:
				#print ('agent%s: region %s reached at %s s\n' 
				#	%(str(i+1), str(reg), str(tt)))
				if Planner[i].segment == 'loop':
					if f[i] == 0:
						print ('agent%s finished its task\n' 
							%(str(i+1)))
						f[i] =1
				Planner[i].find_next_move()
		else:
			# actions
			act = todo[:]
			if act in LOCAL:
				# local action
				if K[i]<=0:
					if act != ACT[i]:
						# new local action
						#print ('agent%s now doing local action %s\n' 
						#		%(str(i+1), str(act)))
						K[i] = LOCAL[act]/STEP
						ACT[i] = act
					else:
						# current action done
						print ('agent%s: action %s done at %s s\n' 
							%(str(i+1), str(act), str(tt)))
						Planner[i].find_next_move()
						if Planner[i].segment == 'loop':
							if f[i] == 0:
								print ('agent%s finished its task\n' 
									%(str(i+1)))
								f[i] = 1
				else:
					K[i] -= 1
					if Planner[i].contract_time>0:
						Planner[i].contract_time -= STEP
					u1 = STEP*random.gauss(0,STEP) 
					u2 = STEP*random.gauss(0,STEP)
					X[i][0] += u1*STEP
					X[i][1] += u2*STEP 
			else:
				# cooperative or assisting actions			
				if K[i]<=0:
					if act != ACT[i]:
						# new action
						if act in dep:
							print ('agent%s now doing cooperative action %s\n' 
								%(str(i+1), str(act)))
						else:
							print ('agent%s now doing assisting action %s\n' 
								%(str(i+1), str(act)))
							print Planner[i].contract_time
						if Planner[i].contract_time>0:
							K[i] = int(round(Planner[i].contract_time/STEP))
							ACT[i] = act
						else:
							print Planner[i].contract_time
							print 'not contract time, it has to be delayed'
							# delay the cooperation by wait
							Planner[i].delay_cooperation(DELAY, SPEED[i])
					else:
						# current action done
						#print Planner[i].contract_time
						print '********************************'
						print '********************************'
						print ('agent%s: action %s done at %s s\n' 
							%(str(i+1), str(act), str(tt)))
						print '********************************'
						print '********************************'
						if Planner[i].segment == 'loop':
							if f[i] == 0:
								print ('agent%s finished its task\n' 
									%(str(i+1)))
								f[i] = 1
						Planner[i].find_next_move()
				else:
					K[i] -= 1
					if Planner[i].contract_time>0:
						Planner[i].contract_time -= STEP
					u1 = STEP*random.gauss(0,STEP)
					u2 = STEP*random.gauss(0,STEP)
					X[i][0] += u1*STEP
					X[i][1] += u2*STEP 
		#======================
		# cooperation request
		if (T[i]<=-10):
			#print 'Planner[i].contract_time', Planner[i].contract_time
			#print 'T[i]', T[i]
			Request[i] = Planner[i].cooperative_action_in_horizon(dep,
					horizon[i])
		T[i] -= 1
	#print Request
	#======================
	# choose one agent to serve
	agents_req = [a for a,b in Request.iteritems() if b]
	if agents_req:
		print '**************'
		print 'agents requested at time %s:' %tt, agents_req
		choosen_agent = random.choice(agents_req)
		choosen_agent = agents_req[0]
		request = Request[choosen_agent]
		print 'choosen', choosen_agent, 'on', request
		print '**************'
		for i in xrange(0,6):
			if (i!=choosen_agent):
				Reply[i] = Planner[i].evaluate_request(request,
					alpha=1)
				#print 'reply from agent %s' %str(i+1) , Reply[i]
		Confirm, time = Planner[choosen_agent].confirmation(request, 
			Reply)
		print 'Confirmation',Confirm
		#print 'choosen_agent contract time', Planner[choosen_agent].contract_time
		#print '**************'
		if time>0:
			for i in xrange(0,6):
				if (i!=choosen_agent):
					Planner[i].adapt_plan(Confirm[i])
					T[i] = int(round(Planner[i].contract_time/STEP))
			T[choosen_agent] = int(round(Planner[choosen_agent].contract_time/STEP))	
		else:
			T[choosen_agent]=DELAY*SPEED[choosen_agent]
	#print 'T', T
	t=t+1
	tt = t*STEP
	if all(item>0 for item in f):
		break
XX = list()
for i in xrange(0,6):
	XX.append(Planner[i].traj)
#fig1 = visualize_dist(DD, STEP, L=10)
#save_pdf('safe_distance_static', fig1)
#fig2 = visualize_traj(Region, XX, L=10)
#save_pdf('safe_traj_static', fig2)
#fig3 = reach_events(Reach_time, Color,t,STEP)
#pyplot.show()
#save_pdf('safe_reach_static', fig3)
#movie_clips(Region, XX, L=50)
fig1 = visualize_traj(Region, XX, L=10, Ed=50)
save_pdf('1a', fig1)
fig2 = visualize_traj(Region, XX, L=10, Ed=120)
save_pdf('2a', fig2)
fig3 = visualize_traj(Region, XX, L=10, Ed=200)
save_pdf('3a', fig3)
fig4 = visualize_traj(Region, XX, L=10, Ed=330)
save_pdf('4a', fig4)
fig5 = visualize_traj(Region, XX, L=10, Ed=430)
save_pdf('5a', fig5)
fig6 = visualize_traj(Region, XX, L=10, Ed=670)
save_pdf('6a', fig6)