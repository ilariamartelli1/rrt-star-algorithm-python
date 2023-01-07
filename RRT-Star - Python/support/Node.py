from numpy import size
from numpy.random import randint
from support.variables import screen_width
from support.variables import screen_height
from support.variables import scale

class Node:
	def __init__(self,pos,cost,node_parent,index):
		if type(pos) == str: # case: pos = 'random'
			x = randint((1-scale)*screen_width, scale*screen_width, 1)[0]
			y = randint((1-scale)*screen_height, scale*screen_height, 1)[0]
		else:
			x,y = pos
		self.x = round(x)
		self.y = round(y)
		self.cost = round(cost)
		self.parent = node_parent
		self.index = index

def reshuffle_init_nodes_if_in_Cobs(V,Cobs):
	do, count = True, 0
	while do: # V init must not be in Cobs
		for i in range(size(V)):
			for j in range(size(Cobs)):
				if V[i].x >= Cobs[j].x and V[i].x <= Cobs[j].x + Cobs[j].lx and V[i].y >= Cobs[j].y and V[i].y <= Cobs[j].y + Cobs[j].ly:
					V[i] = Node('random',0,'none',0) # reshuffle
					count += 1
		if count == 0:	do = False # no further reshuffle needed
		else:			count = 0
	return V