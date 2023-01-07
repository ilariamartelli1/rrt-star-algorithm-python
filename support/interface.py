import pygame
from numpy import size
from numpy import absolute
from support.Target import Target
from support.Obstacle import Obstacle

def check_obstacles_and_target_and_qstart(Cobs,target,pos_node_start,pos_dim,mux,is_target,is_qstart):
	x,y,lx,ly = pos_dim
	start_init = False
	if pygame.key.get_pressed()[pygame.K_t] and is_target == False:
		is_target = True
	if pygame.key.get_pressed()[pygame.K_q] and is_target == True:
		is_qstart = True
	if mux == 1 and pygame.mouse.get_pressed()[0] == True: # left click
		x, y = pygame.mouse.get_pos()
		if is_qstart == True:
			pos_node_start = [x,y]
			start_init = True
		mux = 0
	elif mux == 0 and pygame.mouse.get_pressed()[0] == False and is_qstart == False:
		x2,y2 = pygame.mouse.get_pos()
		lx, ly = x2 - x, y2 - y
		do_not_add = False
		for i in range(size(Cobs)):
			if (x >= Cobs[i].x and x <= Cobs[i].x + Cobs[i].lx) and (y >= Cobs[i].y and y <= Cobs[i].y + Cobs[i].ly):
				do_not_add = True # if first point in an obstacle, cancel
				break
			elif x >= Cobs[i].x - lx and x <= Cobs[i].x + Cobs[i].lx and y >= Cobs[i].y and y <= Cobs[i].y + Cobs[i].ly: # left
				lx = Cobs[i].x - x + 1
			elif y >= Cobs[i].y - ly and y <= Cobs[i].y + Cobs[i].ly and x >= Cobs[i].x and x <= Cobs[i].x + Cobs[i].lx: # up
				ly = Cobs[i].y - y + 1
			elif x >= Cobs[i].x and x <= Cobs[i].x + Cobs[i].lx - lx and y >= Cobs[i].y and y <= Cobs[i].y + Cobs[i].ly: # right
				lx = (Cobs[i].x + Cobs[i].lx) - x
			elif y >= Cobs[i].y and y <= Cobs[i].y + Cobs[i].ly - ly and x >= Cobs[i].x and x <= Cobs[i].x + Cobs[i].lx: # down
				ly = (Cobs[i].y + Cobs[i].ly) - y
			elif x >= Cobs[i].x - lx and x <= Cobs[i].x + Cobs[i].lx - lx and  y >= Cobs[i].y - ly and y <= Cobs[i].y + Cobs[i].ly - ly: # diagonal
				if x <= Cobs[i].x:
					obstacle_angle_x = Cobs[i].x
				else:
					obstacle_angle_x = Cobs[i].x + Cobs[i].lx - 1
				if y <= Cobs[i].y:
					obstacle_angle_y = Cobs[i].y
				else:
					obstacle_angle_y = Cobs[i].y + Cobs[i].ly - 1
				if absolute((x + lx - 1) - obstacle_angle_x) <= absolute((y + ly - 1) - obstacle_angle_y):
					lx = obstacle_angle_x - x + 1
				else:
					ly = obstacle_angle_y - y + 1
		if lx < 0: # makes so that x,y are in the upper left corner and lx,ly are positive
			x = x + lx - 1
			lx = -lx + 2
		if ly < 0:
			y = y + ly - 1
			ly = -ly + 2
		for i in range(size(Cobs)): # checks if the last obstacle includes parts of older obstacles and modifies the latter
			if Cobs[i].x > x and Cobs[i].x + Cobs[i].lx < x + lx and Cobs[i].y > y and Cobs[i].y + Cobs[i].ly < y + ly:
				do_not_add = True # if older is completely included in newer, cancel
				break
			elif Cobs[i].x > x and Cobs[i].x < x + lx and Cobs[i].y > y and Cobs[i].y + Cobs[i].ly < y + ly: # left
				Cobs[i].lx = Cobs[i].x + Cobs[i].lx - (x + lx) + 1
				Cobs[i].x = x + lx - 1
			elif Cobs[i].x > x and Cobs[i].x + Cobs[i].lx < x + lx and Cobs[i].y > y and Cobs[i].y < y + ly: # up
				Cobs[i].ly = Cobs[i].y + Cobs[i].ly - (y + ly) + 1
				Cobs[i].y = y + ly - 1
			elif Cobs[i].x + Cobs[i].lx > x and Cobs[i].x + Cobs[i].lx < x + lx and Cobs[i].y > y and Cobs[i].y + Cobs[i].ly < y + ly: # right
				Cobs[i].lx = x - Cobs[i].x + 1
			elif Cobs[i].x > x and Cobs[i].x + Cobs[i].lx < x + lx and Cobs[i].y + Cobs[i].ly > y and Cobs[i].y + Cobs[i].ly < y + ly: # down
				Cobs[i].ly = y - Cobs[i].y + 1
		if do_not_add == False:
			if is_target == True:
				target.append(Target([x,y],[lx,ly]))
			else:
				Cobs.append(Obstacle([x,y],[lx,ly]))
				# Cobs = check_obstacles_if_overlap(Cobs) # deletes last obstacle if it includes other obstacles
		mux = 1
	pos_dim = x,y,lx,ly
	return Cobs,target,pos_node_start,pos_dim,mux,is_target,is_qstart,start_init


def draw_obstacles_and_target_and_qstart(screen,Cobs,target,pos_dim,mux,is_target,is_qstart):
	x,y,lx,ly = pos_dim
	if mux == 0:
		x2,y2 = pygame.mouse.get_pos()
		lx, ly = x2 - x, y2 - y
		if is_qstart == False: # draw rect while they are clicked
			if is_target == True:
				pygame.draw.rect(screen, 'green', pygame.Rect(x,y,lx,ly), 1)
			else:
				pygame.draw.rect(screen, 'red', pygame.Rect(x,y,lx,ly), 1)
	for i in range(size(Cobs)):
		pygame.draw.rect(screen, 'red', pygame.Rect(Cobs[i].x,Cobs[i].y,Cobs[i].lx,Cobs[i].ly), 1)
	for i in range(size(target)):
		pygame.draw.rect(screen, 'green', pygame.Rect(target[i].x,target[i].y,target[i].lx,target[i].ly), 1)