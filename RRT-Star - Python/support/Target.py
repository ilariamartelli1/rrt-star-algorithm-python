from numpy import size
from numpy.random import randint
from support.variables import screen_width
from support.variables import screen_height
from support.variables import scale

class Target:
	"""#(x,y) = upper left corner of a rectangle"""
	def __init__(self,pos,dim):
		if type(pos) == str: # case: pos = 'random'
			x = randint((1-scale)*screen_width, scale*screen_width, 1)[0]
			y = randint((1-scale)*screen_height, scale*screen_height, 1)[0]
		else:
			x,y = pos
		if type(dim) == str: # case: dim = 'random'
			lx = randint(20, 100, 1)[0]
			ly = randint(20, 100, 1)[0]
		else:
			lx,ly = dim
		self.x = round(x)
		self.y = round(y)
		self.lx = round(lx)
		self.ly = round(ly)

def reshuffle_target_if_overlap(target,Cobs):
	do, count = True, 0
	while do: # Obstacles must not overlap
		for i in range(size(target)):
			for j in range(size(Cobs)):
				if target[i].x >= Cobs[j].x - target[i].lx and target[i].x <= Cobs[j].x + Cobs[j].lx and target[i].y >= Cobs[j].y - target[i].ly and target[i].y <= Cobs[j].y + Cobs[j].ly:
					target[i] = Target('random','random') # reshuffle
					count += 1
		if count == 0:	do = False # no further reshuffle needed
		else:			count = 0
	for i in range(size(target)): # also changes lx and ly to fit the space C
		if target[i].x + target[i].lx > scale*screen_width:
			target[i].lx = scale*screen_width - target[i].x
		if target[i].y + target[i].ly > scale*screen_height:
			target[i].ly = scale*screen_height - target[i].y
	return target