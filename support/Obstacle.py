from numpy import size
from numpy.random import randint
from support.variables import screen_width
from support.variables import screen_height
from support.variables import scale

class Obstacle:
	"""#(x,y) = upper left corner of a rectangle"""
	def __init__(self,pos,dim):
		if type(pos) == str: # case: pos = 'random'
			x = randint((1-scale)*screen_width, scale*screen_width, 1)[0]
			y = randint((1-scale)*screen_height, scale*screen_height, 1)[0]
		else:
			x,y = pos
		if type(dim) == str: # case: dim = 'random'
			lx = randint(50, 300, 1)[0]
			ly = randint(50, 300, 1)[0]
		else:
			lx,ly = dim
		self.x = round(x)
		self.y = round(y)
		self.lx = round(lx)
		self.ly = round(ly)

def reshuffle_obstacles_if_overlap(Cobs):
	do, count = True, 0
	while do: # Obstacles must not overlap
		for i in range(size(Cobs)):
			for j in range(size(Cobs)):
				if i != j and Cobs[j].x >= Cobs[i].x - Cobs[j].lx and Cobs[j].x <= Cobs[i].x + Cobs[i].lx and Cobs[j].y >= Cobs[i].y - Cobs[j].ly and Cobs[j].y <= Cobs[i].y + Cobs[i].ly:
					Cobs[j] = Obstacle('random','random') # reshuffle
					count += 1
		if count == 0:	do = False # no further reshuffle needed
		else:			count = 0
	for i in range(size(Cobs)): # also changes lx and ly to fit the space C
		if Cobs[i].x + Cobs[i].lx > scale*screen_width:
			Cobs[i].lx = scale*screen_width - Cobs[i].x
		if Cobs[i].y + Cobs[i].ly > scale*screen_height:
			Cobs[i].ly = scale*screen_height - Cobs[i].y
	return Cobs