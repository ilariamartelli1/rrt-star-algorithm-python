from numpy import size
from numpy import sqrt
from numpy import power

def distance_points(point1,point2):
	x1,y1 = point1[0],point1[1]
	x2,y2 = point2[0],point2[1]
	return sqrt(power(x1 - x2, 2) + power(y1 - y2, 2))

def distance_nodes(node1,node2):
	x1,y1 = node1.x,node1.y
	x2,y2 = node2.x,node2.y
	return sqrt(power(x1 - x2, 2) + power(y1 - y2, 2))

def points2line(point1,point2):
	"""#implicit form: a*x + b*y + c = 0 - returns a,b,c"""
	x1,y1 = point1
	x2,y2 = point2
	if x1 == x2 and y1 == y2:
		return 0,1,-y1 # 'Error: same point' - AL POSTO DELL'ERRORE MI RICONDUCO AL CASO ORIZZONTALE
	if x1 == x2:	# vertical
		return 1,0,-x1
	m = (y1-y2)/(x1-x2)
	q = y1 - m * x1
	return -m,1,-q

def intersect_point_line_perpendicular(point,line):
	"""#returns the coordinates of the point that intersects the line and the perpendicular passing through the point"""
	xp,yp = point
	a,b,c = line
	if b == 0:		# vertical
		x = -c
		y = yp
		return x,y
	m,q = -a/b,-c/b
	x = (xp + m * yp - m * q)/(m**2 + 1)
	y = m * x + q
	return x,y

def distance_point_line(point,line): # not used
	xi,yi = intersect_point_line_perpendicular(point,line)
	xp,yp = point
	return distance_points([xi,yi], [xp,yp])

def distance_edge(pointP,point1,point2):
	"""#returns -1 if the intersection is out of bounds (out of the edge or on a node)"""
	xp,yp = pointP
	x1,y1 = point1
	x2,y2 = point2
	line = points2line(point1,point2)
	xi,yi = intersect_point_line_perpendicular(pointP,line)
	if ((xi >= x1 and xi >= x2) or (xi <= x1 and xi <= x2)) and ((yi >= y1 and yi >= y2) or (yi <= y1 and yi <= y2)):
		return -1 # intersection out of bounds, distance invalid
	return distance_points([xi,yi], [xp,yp])

def compare_angular_coeff(line1,line2):
	"""#returns 1 if line1's m is greater than line2's m, 0 otherwise
		#also returns -1 if the lines are vertical"""
	a1,b1,c1 = line1
	a2,b2,c2 = line2
	b1, b2 = -b1, -b2 # tiene conto del fatto che l'asse y è verso il basso
	if b1 == 0 or b2 == 0: return -1
	m1,m2 = -a1/b1, -a2/b2
	if m1 > m2: return 1
	else: return 0

def find_intersect(line1,line2):
	"""#returns coordintates of intersect
		#if the lines are parallel, returns [-1,-1]"""
	a1,b1,c1 = line1
	a2,b2,c2 = line2
	if (a2*b1 - a1*b2) == 0: return -1,-1
	x = (b2*c1 - b1*c2)/(a2*b1 - a1*b2) # (c1/b1 - c2/b2)/(a2/b2 - a1/b1)
	if b1 == 0:		# vertical edge
		y = - (a2*x + c2)/b2
	else:
		y = - (a1*x + c1)/b1
	return round(x),round(y)