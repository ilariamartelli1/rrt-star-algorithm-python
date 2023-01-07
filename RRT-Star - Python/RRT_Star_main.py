#!/usr/bin/env python

# --------------------------------------------------------------------------- #
#            New nodes are attached only to nodes, never to edges             #
# --------------------------------------------------------------------------- #
# draw obstacles with mouse, then press t (target) and draw target with mouse #
# when finished, press q and click to enter qstart and run the algorithm      #
# --------------------------------------------------------------------------- #
# if no obstacles or target initialized, default random: 5 obstacles 1 target #
# --------------------------------------------------------------------------- #
#        click space during execution to print current number of nodes        #
# --------------------------------------------------------------------------- #

from support.imports_RRT_Star import *


def init_pygame():
	pygame.init()
	pygame.display.set_caption("RRT*")
	screen = pygame.display.set_mode((screen_width,screen_height))
	clock = pygame.time.Clock()
	return screen,clock


def init_RRT_Star(Cobs,target,pos_node_start):
	V = [1]
	E = []
	G = Graph(V,pos_node_start,E,Cobs,target)
	print(f'node_start = {G.V[0].x},{G.V[0].y}')
	vol_Cobs = 0
	for i in range(size(Cobs)):
		vol_Cobs += G.Cobs[i].lx * G.Cobs[i].ly
	vol_Cfree = scale*screen_width * scale*screen_height - vol_Cobs
	d, zeta_d = 2, pi
	gamma_RRT_Star = 2*power(1 + 1/d, 1/d) * power(vol_Cfree/zeta_d, 1/d)
	if isnan(gamma_RRT_Star):
		print("gamma_RRT_Star isnan: True")
	return G,gamma_RRT_Star,d


def RRT_Star(G,radius_V,nodes_to_target):
	q_rand = G.generate_node('random', nodes_to_target, radius_V)
	q_nearest = G.nearest(q_rand)
	q_new = G.steer(q_nearest,q_rand,eta)
	q_new_in_Cfree = G.stopping_configuration(q_nearest, q_new)
	if q_new_in_Cfree.x != q_nearest.x or q_new_in_Cfree.y != q_nearest.y:
		q_new = q_new_in_Cfree
		q_new.cost = q_nearest.cost + distance_nodes(q_nearest, q_new)
		G.add_vertex(q_new)
		q_min = q_nearest
		c_min = q_new.cost
		Q_near = G.near(q_new, radius_V)
		for i in range(size(Q_near)):
			q_near = Q_near[i]
			c = q_near.cost + distance_nodes(q_near, q_new)
			if G.obstacle_free(q_near, q_new) and c < c_min:
				q_min = q_near
				c_min = c
		q_new.parent = q_min
		q_new.cost = c_min
		G.add_edge(q_min, q_new)
		for i in range(size(Q_near)): # rewiring Q_near
			q_near = Q_near[i]
			c = q_new.cost + distance_nodes(q_near, q_new)
			if G.obstacle_free(q_near, q_new) and c < q_near.cost:
				q_parent = q_near.parent	# old parent
				q_near.parent = q_new		# new parent
				q_near.cost = c
				G.add_edge(q_new, q_near)
				for j in range(G.sizeE):
					par1 = G.E[j].node1.index == q_parent.index
					par2 = G.E[j].node2.index == q_parent.index
					near1 = G.E[j].node1.index == q_near.index
					near2 = G.E[j].node2.index == q_near.index
					if (par1 and near2) or (near1 and par2):
						G.remove_edge(j)
						break


if __name__ == '__main__':
	loop = True
	screen,clock = init_pygame()
	eta = 100
	mux = 1
	Cobs = []
	target = []
	nodes_to_target = []
	pos_node_start = [50,50] # default
	x,y,lx,ly = 0,0,0,0
	pos_dim = x,y,lx,ly
	is_target = False
	is_qstart = False
	start_algorithm = False
	while loop:
		for event in pygame.event.get():
			if event.type == pygame.QUIT or pygame.key.get_pressed()[pygame.K_ESCAPE]:
				loop = False
		
		screen.fill((0, 0, 0)) # screen refresh

		if start_algorithm == False:
			Cobs,target,pos_node_start,pos_dim,mux,is_target,is_qstart,start_init = check_obstacles_and_target_and_qstart(Cobs,target,pos_node_start,pos_dim,mux,is_target,is_qstart)
			draw_obstacles_and_target_and_qstart(screen,Cobs,target,pos_dim,mux,is_target,is_qstart)
		
		if start_init == True:
			G, gamma_RRT_Star, d = init_RRT_Star(Cobs,target,pos_node_start)
			start_init = False
			start_algorithm = True

		if start_algorithm == True:
			radius_V = min(gamma_RRT_Star * power(log(G.sizeV)/G.sizeV, 1/d), eta) # min with eta to avoid too large radius
			RRT_Star(G, radius_V, nodes_to_target)
			nodes_to_target = G.check_target()
			G.draw(screen, nodes_to_target)
			if pygame.key.get_pressed()[pygame.K_SPACE]:
				print('') # carriage return
				print('number of nodes = ', G.sizeV)
				print('radius_V = ', round(radius_V))
				print('cost of road to target = ', round(G.current_cost))
				# print('best cost found = ', round(G.best_cost))
				print('prob = ', round(start_prob-G.adj_prob))
			if pygame.key.get_pressed()[pygame.K_a]:
				adjust_time -= 1
			if pygame.key.get_pressed()[pygame.K_s]:
				adjust_time += 1

		pygame.display.flip()
		# clock.tick(adjust_time)
pygame.quit()