from numpy import size
from numpy.random import randint
from pygame import Rect
from pygame.draw import rect
from pygame.draw import circle
from pygame.draw import line
from pygame.font import SysFont
from support.Target import Target
from support.Target import reshuffle_target_if_overlap
from support.Obstacle import Obstacle
from support.Obstacle import reshuffle_obstacles_if_overlap
from support.Node import Node
from support.Node import reshuffle_init_nodes_if_in_Cobs
from support.Edge import Edge
from support.functions import distance_points
from support.functions import distance_edge
from support.functions import intersect_point_line_perpendicular
from support.functions import distance_nodes
from support.functions import points2line
from support.functions import find_intersect
from support.functions import compare_angular_coeff
from support.variables import screen_width
from support.variables import screen_height
from support.variables import scale
from support.variables import check_edge
from support.variables import start_prob

class Graph:
	def __init__(self,V,pos_node_start,E,Cobs,target): # Vertices, Edges
		print('Graph = (V=' + str(V) + ', E=' + str(E) + ')')
		if size(Cobs) == 0: # Cobs not initialized
			Cobs = [Obstacle('random','random') for i in range(5)]
			Cobs = reshuffle_obstacles_if_overlap(Cobs)
		if size(target) == 0: # target not initialized
			target = [Target('random','random') for i in range(1)]
			target = reshuffle_target_if_overlap(target,Cobs)
		sizeV = 1
		V = [Node(pos_node_start,0,'none',0)] # Cost(node_start) = 0, Parent(node_start) = 'none', Index(node_start) = 0
		V = reshuffle_init_nodes_if_in_Cobs(V,Cobs)
		sizeE = 0
		E = []
		self.V = V
		self.E = E
		self.sizeV = sizeV
		self.sizeE = sizeE
		self.Cobs = Cobs
		self.target = target
		self.progressive_number = -1	# used in generate_node and check_target
		self.current_cost = -1		# used in check_target
		self.best_cost = -1				# used in check_target
		self.adj_prob = start_prob		# used in generate_node

	def generate_node(self,coord,nodes_to_target,radius_V):
		"""#generate random node, or node with given coordinates, or node near current path found"""
		cost = -1
		parent = -1
		index = self.sizeV
		pn = self.progressive_number
		prob,adj_prob = 100,0
		size_path = size(nodes_to_target)
		if size_path != 0:
			prob = randint(0, 100, 1)[0]
			self.adj_prob = (self.sizeV - pn)*0.1
			if self.adj_prob > start_prob: self.adj_prob = start_prob
			adj_prob = self.adj_prob
		if type(coord) == str and prob > start_prob-adj_prob:	# case: coord = 'random'
			node_new = Node('random',cost,parent,index)
		elif type(coord) == str and prob <= start_prob-adj_prob: # start_prob = 60% (or less) chance of a random point near the found path (within +-radius_V of the node coordinates)
			i = randint(0, size_path-1, 1)[0] # picks one of the nodes in the path with uniform probability
			px, py = randint(-radius_V,radius_V,1)[0], randint(-radius_V,radius_V,1)[0]
			x, y = self.V[nodes_to_target[i]].x + px, self.V[nodes_to_target[i]].y + py
			if   x < (1-scale)*screen_width:  x = 0
			elif x > scale*screen_width:	  x = scale*screen_width
			if   y < (1-scale)*screen_height: y = 0
			elif y > scale*screen_height:	  y = scale*screen_height
			Cobs = self.Cobs
			for j in range(size(Cobs)):
				if x >= Cobs[j].x and x <= Cobs[j].x + Cobs[j].lx and y >= Cobs[j].y and y <= Cobs[j].y + Cobs[j].ly: # if inside one of the obstacles
					node = self.stopping_configuration(self.V[nodes_to_target[i]], Node([x,y],cost,parent,index))
					x, y = node.x, node.y
					break
			node_new = Node([x,y],cost,parent,index)
		else:
			node_new = Node(coord,cost,parent,index)
		return node_new
	
	def add_vertex(self,node):
		"""#adds 1 vertex"""
		self.V.append(node)
		self.sizeV += 1

	def add_edge(self,node1,node2):
		"""#uses Node classes"""
		if type(node2) == str: # case: node2 = 'last'
			node2 = self.V[self.sizeV-1]
		self.E.append(Edge(node1,node2))
		self.sizeE += 1
	
	def remove_edge(self,i):
		"""#uses index"""
		del self.E[i]
		self.sizeE -= 1
	
	def nearest(self,node_new):
		"""#returns the nearest node"""
		pointP = node_new.x, node_new.y
		V,sizeV = self.V,self.sizeV
		index,d = -1,-1
		edge_win = 0
		for i in range(sizeV): # computes distance nodes
			pointI = V[i].x, V[i].y # x,y di i
			d_vertex = distance_points(pointP, pointI)
			if d_vertex < d or d == -1:
				d = d_vertex
				index = i
		index_q_attach = index
		return self.V[index_q_attach]
	
	def nearest_check_edge(self,node_new):
		"""#returns the nearest node if it is an existing node, or if it is on an edge (one more vertex created)"""
		pointP = node_new.x, node_new.y
		V,E,sizeV,sizeE = self.V,self.E,self.sizeV,self.sizeE
		index,d = -1,-1
		edge_win = 0
		new = 1
		for i in range(sizeV): # computes distance nodes
			pointI = V[i].x, V[i].y # x,y di i
			d_vertex = distance_points(pointP, pointI)
			if d_vertex < d or d == -1:
				d = d_vertex
				index = i
		if check_edge:
			for i in range(sizeE): # computes distance edges
				point1 = E[i].node1.x,E[i].node1.y
				point2 = E[i].node2.x,E[i].node2.y
				d_edge = distance_edge(pointP, point1, point2)
				if d_edge < d and d_edge != -1:
					d = d_edge
					index = i
					edge_win = 1
			if edge_win: # case: min distance on edge
				i = index
				point1 = E[i].node1.x,E[i].node1.y
				point2 = E[i].node2.x,E[i].node2.y
				x,y = intersect_point_line_perpendicular(pointP, points2line(point1, point2))
				node = self.generate_node([x,y])
				self.add_vertex(node) # modifies self.V and self.sizeV
				index = self.sizeV-1
				cost1 = self.E[i].node1.cost + distance_nodes(self.E[i].node1, self.V[index])
				cost2 = self.E[i].node2.cost + distance_nodes(self.E[i].node2, self.V[index])
				if cost1 < cost2:
					self.V[index].cost = cost1
					self.V[index].parent = self.E[i].node1
				else:
					self.V[index].cost = cost2
					self.V[index].parent = self.E[i].node2
				self.add_edge(E[i].node1, V[index]) # V[index] here is the vertex on the edge
				self.add_edge(E[i].node2, V[index])
				self.remove_edge(i)
				new = 2
		index_q_attach = index
		return self.V[index_q_attach]
	
	def near(self,node_new,r):
		"""#returns all the nodes nearer than radius r to node_new"""
		pointP = node_new.x, node_new.y
		V,E,sizeV,sizeE = self.V,self.E,self.sizeV,self.sizeE
		U = []
		for i in range(sizeV): # computes distance nodes
			pointI = V[i].x, V[i].y # x,y di i
			d_vertex = distance_points(pointP, pointI)
			if d_vertex < r:
				U.append(V[i])
		return U

	def check_Cfree(self,node): # not used
		"""#if node in Cfree returns -1, if in Cobs returns index of Obstacle; Obstacle border not considered part of the Obstacle"""
		x,y = node.x,node.y
		Cobs = self.Cobs
		for i in range(size(Cobs)):
			xo,yo,lx,ly = Cobs[i].x,Cobs[i].y,Cobs[i].lx,Cobs[i].ly
			if (x > xo and x < xo+lx) and (y > yo and y < yo+ly):
				return i
		return -1
	
	def stopping_configuration(self,node_attach,node_new):
		"""#if node_new is in Cobs or if its edge passes throught Cobs, generates node_new_in_Cfree on the line between node_new and node_attach
			#if node_attach is in the 4 adjacent quadrants of the obstacle, the obstacle side that intersects is the corrispective
			#if node_attach is in the 4 corner quadrants with respect to the obstacle, checks the line that passes throught node_new and the corrispective corner of the obstacle and compares the m of the 2 lines
		"""
		node_new_in_Cfree = node_new
		x,y = node_attach.x,node_attach.y
		point_new = [node_new.x,node_new.y]
		dist = distance_points([x,y], point_new) # distance node_attach -- node_new
		line_edge = points2line([x,y], point_new)
		for i in range(size(self.Cobs)): # checks every Obstacle for edge overlaps
			obs = self.Cobs[i]
			x1,y1,x4,y4 = obs.x, obs.y, obs.x+obs.lx, obs.y+obs.ly # obstacle coordinates (upper left, lower right)
			if (node_attach.x <= x1 and node_new.x <= x1) or (node_attach.x >= x4 and node_new.x >= x4) or (node_attach.y <= y1 and node_new.y <= y1) or (node_attach.y >= y4 and node_new.y >= y4):
				continue # if node_attach and node_new are both on one side with respect to the obstacle, skip
			else:
				ul,ur,dl,dr = [x1,y1], [x4,y1], [x1,y4], [x4,y4] # obstacle corners (up left, up right, down left, down right)
				left,above,right,below = [ul,dl], [ul,ur], [ur,dr], [dl,dr] # obstacle sides
				side = [left, above, right, below]
				# first check adjacent, then corners
						# adjacent
				if   y <= y1 and x >= x1 and x <= x4: p1,p2 = above	# adjacent, above
				elif y >= y4 and x >= x1 and x <= x4: p1,p2 = below	# adjacent, below
				elif x <= x1 and y >= y1 and y <= y4: p1,p2 = left	# adjacent, left
				elif x >= x4 and y >= y1 and y <= y4: p1,p2 = right	# adjacent, right
				# corners
				else:
					if   x < x1 and y < y1: c,s = ul,0				# corner up left
					elif x > x4 and y < y1: c,s = ur,1				# corner up right
					elif x > x4 and y > y4: c,s = dr,2				# corner down right
					elif x < x1 and y > y4: c,s = dl,3				# corner down left
					line_check = points2line(c, point_new)
					i = compare_angular_coeff(line_check, line_edge)
					if i == -1: continue # ignores iteration if vertical lines in corner
					s = s + i
					if s > 3: s = 0
					p1,p2 = side[s]
				line_obs = points2line(p1, p2)
				xi,yi = find_intersect(line_edge, line_obs)
				if xi == -1: continue # ignores iteration if the lines are parallel
				dist_new = distance_points([x,y], [xi,yi]) # distance node_attach -- intersection
				if xi >= x1 and xi <= x4 and yi >= y1 and yi <= y4 and dist_new < dist:
					node_new_in_Cfree = self.generate_node([xi,yi],[],0) # [] and 0 becaouse it doesn't need nodes_to_target and radius_V
					dist = dist_new
		return node_new_in_Cfree
	
	def obstacle_free(self,node_attach,node_new):
		"""#returns 1 if yes, 0 if no ("node_attach" in code doesn't refer to anything, don't want to change it from previous version)"""
		x,y = node_attach.x,node_attach.y
		point_new = [node_new.x,node_new.y]
		line_edge = points2line([x,y], point_new)
		for i in range(size(self.Cobs)): # checks every Obstacle for edge overlaps
			obs = self.Cobs[i]
			x1,y1,x4,y4 = obs.x, obs.y, obs.x+obs.lx, obs.y+obs.ly # obstacle coordinates (upper left, lower right)
			if (node_attach.x <= x1 and node_new.x <= x1) or (node_attach.x >= x4 and node_new.x >= x4) or (node_attach.y <= y1 and node_new.y <= y1) or (node_attach.y >= y4 and node_new.y >= y4):
				continue # if node_attach and node_new are both on one side with respect to the obstacle, skip
			else:
				ul,ur,dl,dr = [x1,y1], [x4,y1], [x1,y4], [x4,y4] # obstacle corners (up left, up right, down left, down right)
				left,above,right,below = [ul,dl], [ul,ur], [ur,dr], [dl,dr] # obstacle sides
				side = [left, above, right, below]
				# first check adjacent, then corners
						# adjacent
				if   y <= y1 and x >= x1 and x <= x4: p1,p2 = above	# adjacent, above
				elif y >= y4 and x >= x1 and x <= x4: p1,p2 = below	# adjacent, below
				elif x <= x1 and y >= y1 and y <= y4: p1,p2 = left	# adjacent, left
				elif x >= x4 and y >= y1 and y <= y4: p1,p2 = right	# adjacent, right
				# corners
				else:
					if   x < x1 and y < y1: c,s = ul,0				# corner up left
					elif x > x4 and y < y1: c,s = ur,1				# corner up right
					elif x > x4 and y > y4: c,s = dr,2				# corner down right
					elif x < x1 and y > y4: c,s = dl,3				# corner down left
					line_check = points2line(c, point_new)
					i = compare_angular_coeff(line_check, line_edge)
					if i == -1: continue # ignores iteration if vertical lines in corner
					s = s + i
					if s > 3: s = 0
					p1,p2 = side[s]
				line_obs = points2line(p1, p2)
				xi,yi = find_intersect(line_edge, line_obs)
				if xi == -1: continue # ignores iteration if the lines are parallel
				if xi >= x1 and xi <= x4 and yi >= y1 and yi <= y4:
					return 0
		return 1
	
	def steer(self,node_attach,node_new,eta):
		"""#finds intersect among the line between node_attach and node_new
			#and a circumference with center in node_new and radius eta"""
		if distance_nodes(node_attach,node_new) <= eta:
			return node_new
		a,b,c = points2line([node_attach.x,node_attach.y],[node_new.x,node_new.y])
		if b != 0:
			m, q = -a/b, -c/b
			x_c, y_c, r = node_attach.x, node_attach.y, eta
			A, B, C = -2*x_c, -2*y_c, x_c**2 + y_c**2 - r**2
			x1 = -(A + B*m + 2*m*q + (A**2 + 2*A*B*m + 4*A*m*q + B**2*m**2 - 4*B*q - 4*C*m**2 - 4*q**2 - 4*C)**(1/2))/(2*(m**2 + 1))
			x2 = -(A + B*m + 2*m*q - (A**2 + 2*A*B*m + 4*A*m*q + B**2*m**2 - 4*B*q - 4*C*m**2 - 4*q**2 - 4*C)**(1/2))/(2*(m**2 + 1))
			y1 = q - (m*(A + B*m + 2*m*q + (A**2 + 2*A*B*m + 4*A*m*q + B**2*m**2 - 4*B*q - 4*C*m**2 - 4*q**2 - 4*C)**(1/2)))/(2*(m**2 + 1))
			y2 = q - (m*(A + B*m + 2*m*q - (A**2 + 2*A*B*m + 4*A*m*q + B**2*m**2 - 4*B*q - 4*C*m**2 - 4*q**2 - 4*C)**(1/2)))/(2*(m**2 + 1))
		else: # vertical
			x1 = node_attach.x
			x2 = x1
			y1 = node_attach.y + eta
			y2 = node_attach.y - eta
		if distance_points([node_new.x,node_new.y],[x1,y1]) < distance_points([node_new.x,node_new.y],[x2,y2]):
			node_new.x, node_new.y = round(x1), round(y1)
		else:
			node_new.x, node_new.y = round(x2), round(y2)
		return node_new

	def check_target(self):
		"""#checks every node in target (if node.parent not in target) and goes backwards to node_start, then chooses the road with the smallest cumulative cost"""
		V,sizeV,target,bc = self.V,self.sizeV,self.target,self.best_cost
		nodes_to_target = [] # since the RRT* algorithm can accidently make a worse path, you can't keep the old one or update only when a better one is found, it has to be updated every time
		index,cost = -1,-1
		for i in range(1,sizeV): # does not check node_start
			for j in range(size(target)):
				and1 = (V[i].x > target[j].x and V[i].x < target[j].x + target[j].lx) and (V[i].y > target[j].y and V[i].y < target[j].y + target[j].ly) # node inside target
				and2 = (V[i].parent.x <= target[j].x or V[i].parent.x >= target[j].x + target[j].lx) or (V[i].parent.y <= target[j].y or V[i].parent.y >= target[j].y + target[j].ly) # parent outside target
				if (and1 and and2) and (self.V[i].cost < cost or cost == -1):
						if round(self.V[i].cost,1) < round(bc,1) or bc == -1: # first path found or new better path found ; could be changed to "if self.V[i].cost < bc-10 or bc == -1:" 
							self.progressive_number = sizeV # resets pn to sizeV (for probability in generate_node)
						cost = self.V[i].cost
						self.current_cost = cost # cumulative cost of the entire road
						if cost < bc or bc == -1:
							self.best_cost = cost # updates bc only if the new path is the best until then
						index = i # i == V[i].index
		if index != -1:
			nodes_to_target.append(index)
			while index != 0: # or "while type(V[index].parent) != str" --> case: V[index].parent = 'none' --> node_start reached
				index = V[index].parent.index
				nodes_to_target.append(index)
		return nodes_to_target
	
	def draw(self,screen,nodes_to_target):
		"""#draws Graph (Nodes, Edges, Obstacles, Targets)"""
		Cobs,V,E,target = self.Cobs,self.V,self.E,self.target
		for i in range(size(Cobs)): # draw obstacles
			rect(screen, 'red', Rect(Cobs[i].x, Cobs[i].y, Cobs[i].lx, Cobs[i].ly), 1)
		font = SysFont(None, 15)
		for i in range(self.sizeE): # draw all edges
			line(screen, 'blue', [ E[i].node1.x, E[i].node1.y ], [ E[i].node2.x, E[i].node2.y ])
		for i in range(self.sizeE): # draw winning edges a different color, needs another for loop because has to be in front of other edges
			for j in range(size(nodes_to_target)-1):
				or1 = E[i].node1.index == nodes_to_target[j] and E[i].node2.index == nodes_to_target[j+1] # nodes_to_target is indices
				or2 = E[i].node1.index == nodes_to_target[j+1] and E[i].node2.index == nodes_to_target[j]
				if or1 or or2:
					line(screen, 'yellow', [ E[i].node1.x, E[i].node1.y ], [ E[i].node2.x, E[i].node2.y ], 2)
					break
		# for i in range(self.sizeV): # draw nodes
		# 	circle(screen, 'red', [ V[i].x, V[i].y ], 1)
			# screen.blit(font.render(str(i), True, 'red'), (V[i].x + 5, V[i].y - 5)) # numbers on the nodes
			# screen.blit(font.render(str(i) + ' ' + str(int(V[i].x)) + ',' + str(int(V[i].y)), True, 'red'), (V[i].x + 5, V[i].y - 5)) # numbers on the nodes + coordinates
		for i in range(size(target)): # draw target
			rect(screen, 'green', Rect(target[i].x, target[i].y, target[i].lx, target[i].ly), 1)
	
	def print_V(self):
		V = self.V
		for i in range(self.sizeV):
			print('V[' + str(i) + '] = (' + str(V[i].x) + ',' + str(V[i].y) + ')')
	
	def print_E(self):
		E = self.E
		for i in range(self.sizeE):
			print('E[' + str(i) + '] = (' + str(E[i].node1.x) + ',' + str(E[i].node1.y) + ')(' + str(E[i].node2.x) + ',' + str(E[i].node2.y) + ')')

	def print_Cobs(self):
		Cobs = self.Cobs
		for i in range(size(Cobs)):
			print('Cobs[' + str(i) + '] = (' + str(Cobs[i].x) + ',' + str(Cobs[i].y) + ')(' + str(Cobs[i].lx) + ',' + str(Cobs[i].ly) + ')')
	