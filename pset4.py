import random as random
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import copy
from scipy.spatial import ConvexHull
from scipy.spatial import convex_hull_plot_2d
from mpl_toolkits.mplot3d import Axes3D
from numpy import cos, sin

#robot dimensions
w=85.0
r=20.0

#Environment:
env_max_x=1000
env_max_y=800


#max iterations
max_ite=3000

#threshold for goal judgement:
THRESHOLD=20.0

#control parameters
Kp_a = 0.3
K_alpha = 1.5
K_beta = -0.3

class node:
	x=0
	y=0
	theta=0
	parent=0

	def __init__(self, x_, y_, theta_):
		self.x=x_
		self.y=y_
		self.theta=theta_

#start node and goal node designate:
start_node = node(100,100,math.pi/2)#starting position and orientation
start_node.parent = 0
#goal_node = node(585,760,math.pi/2)#head in parking 
goal_node = node(680,42.5,0)
goal_node.parent = 10000
#goal_node = node(680,0,0,10000)#along the road parking 

#list of nodes:
node_list=[]

#add start node in the node list:
#node_list.append(start_node)

#obstacle lists
obs=[np.array([300,0,50,400]),
	np.array([500,0,80,85]),
	np.array([780,0,80,85]),
	np.array([400,720,85,80]),
	np.array([685,720,85,80])]

def nearest_node(node_list, q_rand):
	current_min = 10000
	for i in range(len(node_list)):
		node=np.array([node_list[i].x,node_list[i].y,node_list[i].theta])
		dist = np.linalg.norm(q_rand-node,ord=2)
		if dist<current_min:
			current_min = dist
			min_index = i
	return min_index
		

def trajectory_generation(start_state, goal_state):
	x, y, theta = start_state.x, start_state.y, start_state.theta
	x_goal, y_goal, theta_goal = goal_state[0], goal_state[1], goal_state[2]
	trajectory = []
	inputs = []
	dt=0.05
	t=0
	a = math.sqrt((x_goal-x)**2+(y_goal-y)**2)
	trajectory.append(np.array([x,y,theta]))
	while t<=0.3 or a>=0.01:
		a = math.sqrt((x_goal-x)**2+(y_goal-y)**2)
		alpha = (math.atan2((y_goal-y),(x_goal-x))-theta + math.pi)%(2*math.pi)-math.pi
		beta = (theta_goal - theta - alpha + math.pi)%(2*math.pi)-math.pi
		v = Kp_a*a
		omega = K_alpha*alpha+K_beta*beta
		omega_r = (omega*w + 2*v)/(2*r)
		omega_l = (2*v - omega*w)/(2*r)
		inputs.append(np.array([omega_l,omega_r]))
		theta += omega*dt
		x += v*math.cos(theta)*dt
		y += v*math.sin(theta)*dt
		t += dt
		trajectory.append(np.array([x,y,theta]))
		#print("x,y,theta: ", x,y,theta)
	return trajectory #list



def rrt_planner(obstacle_list):
	node_list.append(start_node)
	count = 0
	loop = True
	ite_num = 0
	fig = plt.figure()
	plt.xlim((0,1000))
	plt.ylim((0,800))
	while(loop):
		q_rand=[random.random()*env_max_x, random.random()*env_max_y, random.random()*math.pi*2 - math.pi]

		#find the nearest node for a generated random point in the environment.
		nearest_node_index = nearest_node(node_list, q_rand)
		# expand the tree by given control inputs
		extension = trajectory_generation(node_list[nearest_node_index], q_rand) # this function returns a list with arrays
		# check if the generated trajectory is collision free
		no_collision = collision_check(node_list[nearest_node_index], extension)
		if no_collision:
			new_node=node(extension[-1][0], extension[-1][1], extension[-1][2])
			new_node.parent = nearest_node_index
			node_list.append(new_node)
			count += 1
			draw_traj(node_list[nearest_node_index], new_node, extension)
		ite_num += 1

		#for i in range(len(node_list)):
		dist = cal_dist(node_list[-1],goal_node)
		if dist < THRESHOLD:
			loop = False	

		if ite_num>=max_ite:
			loop = False

	goal_node.parent = len(node_list)-1

	ax = fig.add_subplot(1,1,1)
	ax = visualization_env(ax)
	#plt.show()
	return node_list

def rrt_path():
	node_list = rrt_planner(obs)
	path = []
	count = goal_node.parent# find the connected node searching through index
	path=[]
	path.append(goal_node)
	while count>0:
		print("count: ", count)
		path_node = node_list[count]
		path.append(path_node)
		count = path_node.parent
		print ("path_node x y theta: ", path_node.x, path_node.y, path_node.theta)
		if count == 0:
			print("count: ", count)
			path.append(node_list[count])
			print ("path_node x y theta: ", path_node.x, path_node.y, path_node.theta)
	plot_x=[]
	plot_y=[]
	theta=[]
	for i in range(len(path)-1):
		extension = trajectory_generation(path[len(path)-i-1], np.array([path[len(path)-i-2].x,path[len(path)-i-2].y,path[len(path)-i-2].theta]))
		plt.arrow(path[len(path)-i-1].x, path[len(path)-i-1].y, np.cos(path[len(path)-i-1].theta),
          np.sin(path[len(path)-i-1].theta), color='b', width=1)
		for j in range(len(extension)):
			plot_x.append(extension[j][0])
			plot_y.append(extension[j][1])
			theta.append(extension[j][2])
	plt.plot(plot_x,plot_y,color='red', linewidth=3.0, linestyle='-')
	plt.arrow(path[0].x, path[0].y, np.cos(path[0].theta),
          np.sin(path[0].theta), color='b', width=1)
	plt.show()




def draw_traj(node1, node2, traj):
	plot_x = []
	plot_y = []
	for i in range(len(traj)):
		plot_x.append(traj[i][0])
		plot_y.append(traj[i][1])
	plt.plot(plot_x,plot_y,color='black', linewidth=1.0, linestyle='-')
	#plt.plot(node1.x,node1.y,'bo',markersize='10')
	#plt.plot(node2.x,node2.y,'r*',markersize='10')





def collision_check(nearest_node, extension):
	no_collision = True
	obs_=copy.deepcopy(obs)
	obs_[0][0] -= 42.5
	obs_[0][2] += 85
	obs_[0][3] += 42.5
	obs_[3][0] -= 42.5
	obs_[3][1] -= 42.5
	obs_[3][2] += 85
	obs_[3][3] += 42.5
	obs_[4][0] -= 42.5
	obs_[4][1] -= 42.5
	obs_[4][2] += 85
	obs_[4][3] += 42.5
	obs_[1][0] -= 42.5
	#obs_[1][1] -= 42.5
	obs_[1][2] += 85
	obs_[1][3] += 42.5
	obs_[2][0] -= 42.5
	#obs_[2][1] -= 42.5
	obs_[2][2] += 85
	obs_[2][3] += 42.5
	for j in range(len(obs_)):
		for i in range(len(extension)):
			if (extension[i][0] >= obs_[j][0]) and (extension[i][0] <= (obs_[j][0]+obs_[j][2])) and (extension[i][1] >= obs_[j][1]) and (extension[i][1]<=(obs_[j][1]+obs_[j][3])):
				no_collision = False
			elif extension[i][0] <=0 or extension[i][0] >= env_max_x or extension[i][1] <=0 or extension[i][1] >= env_max_y:
				no_collision = False
	return no_collision

def cal_dist(node1, node2):
	return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2 + (node1.theta - node2.theta)**2)

def visualization_env(ax):
	#fig = plt.figure(figsize = (9, 6))
	#ax = fig.add_subplot(1,1,1)
	#plt.xlim((0, 1000))
	#plt.ylim((0, 800))
	plt.grid()

	#plt.arrow(start_node.x, start_node.y, np.cos(start_node.theta),
          #np.sin(start_state.theta), color='b', width=1)
	#plt.arrow(goal_node.x, goal_node.y, np.cos(goal_node.theta),
          #np.sin(goal_node.theta), color='r', width=1)

	for i in range(len(obs)):
		obstacle = plt.Rectangle([obs[i][0], obs[i][1]], obs[i][2], obs[i][3], edgecolor='g', linewidth=3)
		ax.add_patch(obstacle)
	#plt.show()
	return ax


def draw_Cspace(initial_state, goal_state, obstaclelist):
	""" translate the previous Ospace map into Cspace map using ConvexHull to
	form the C-space obstacles"""

	fig = plt.figure(figsize = (12, 8))
	#ax = fig.gca()
	ax = fig.gca(projection='3d')
	ax.set_xlim([0, 1000])
	ax.set_ylim([0, 800])
	ax.view_init(elev=60)
	Cspace_obstacle = {}
	obstacle_list=[(300,0,50,400),
		(500,0,80,85),
		(780,0,80,85),
		(400,720,85,80),
		(685,720,85,80)]
	for ob in obstacle_list: 
		theta_sample = np.linspace(-np.pi, np.pi, 100)
		points = []
		for theta in theta_sample:
			A = [[ob[0], ob[1], theta], [ob[0]+ob[2], ob[1], theta],
				[ob[0], ob[1]+ob[3], theta], [ob[0]+ob[2], ob[1]+ob[3], theta]]
			for a in A:
				vertice1 = [a[0]+70*cos(theta)+42.5*cos(theta-np.pi/2),
				a[1]+70*sin(theta)+42.5*sin(theta-np.pi/2), theta]
				vertice2 = [a[0]+70*cos(theta)+42.5*cos(theta+np.pi/2),
				a[1]+70*sin(theta)+42.5*sin(theta+np.pi/2), theta]
				vertice3 = [a[0]+10*cos(theta+np.pi)+42.5*cos(theta-np.pi/2),
				a[1]+10*sin(theta+np.pi)+42.5*sin(theta-np.pi/2), theta]
				vertice4 = [a[0]+10*cos(theta+np.pi)+42.5*cos(theta+np.pi/2),
				a[1]+10*sin(theta+np.pi)+42.5*sin(theta+np.pi/2), theta]
				points.append(a)
				points.append(vertice1)
				points.append(vertice2)
				points.append(vertice3)
				points.append(vertice4)

		points = np.array(points)
		hull= ConvexHull(points)
		for i in hull.simplices:
			plt.plot(points[i,0], points[i,1], points[i,2], 'b')
		Cspace_obstacle[ob] = hull
	ax.scatter(initial_state[0], initial_state[1], initial_state[2],
		c = 'r', marker ='o', s=100)
	ax.scatter(goal_state[0], goal_state[1], goal_state[2],
		c = 'g', marker ='o', s=100)
	plt.grid()
	plt.axis('equal')
	plt.show()        
	return Cspace_obstacle


	
rrt_path()

def visualization_env_2(start_node, goal_node):
	fig = plt.figure(figsize = (9, 6))
	ax = fig.add_subplot(1,1,1)
	plt.xlim((0, 1000))
	plt.ylim((0, 800))
	plt.grid()

	plt.arrow(start_node.x, start_node.y, np.cos(start_node.theta),
          np.sin(start_node.theta), color='b', width=1)
	plt.arrow(goal_node.x, goal_node.y, np.cos(goal_node.theta),
          np.sin(goal_node.theta), color='r', width=1)

	for i in range(len(obs)):
		obstacle = plt.Rectangle([obs[i][0], obs[i][1]], obs[i][2], obs[i][3], edgecolor='g', linewidth=3)
		ax.add_patch(obstacle)
	plt.show()
	return ax


def main():
	start_state=node(100,100,math.pi/2)
	goal_state=np.array([585,760,math.pi/2])
	traj=trajectory_generation(start_state,goal_state)
	print("traj[0,:]",len(traj))
	plot_x=[]
	plot_y=[]
	plt.figure()
	for i in range(len(traj)):
		plot_x.append(traj[i][0])
		plot_y.append(traj[i][1])
	plt.plot(plot_x,plot_y,color='black', linewidth=1.0, linestyle='--')
	plt.plot(start_state.x,start_state.y,'bo',markersize='10')
	plt.plot(goal_state[0],goal_state[1],'r*',markersize='10')
	plt.arrow(start_state.x, start_state.y, np.cos(start_state.theta),
          np.sin(start_state.theta), color='b', width=1)
	plt.arrow(goal_state[0], goal_state[1], np.cos(goal_state[2]),
          np.sin(goal_state[2]), color='r', width=1)
	plt.xlim((-100,1000))
	plt.ylim((-100,1000))
	plt.show()
	visualization_env_2(start_node, goal_node)
	draw_Cspace(np.array([start_state.x, start_state.y, start_state.theta]), goal_state, obs)

main()