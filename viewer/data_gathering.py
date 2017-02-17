import math
import pygame
from pygame.locals import *
import random as rand
import numpy as np
import sys
import isovist as iso
from numpy import atleast_2d

from my_rrt import *
'''
Init Screen

Creates a pygame display

Returns a screen and a clock

'''

def InitScreen(xdim, ydim):
	pygame.init()
	pygame.font.init()

	size = (xdim, ydim)
	screen = pygame.display.set_mode(size)

	pygame.display.set_caption("Quixotic")
	clock = pygame.time.Clock()

	return screen, clock

'''
	Updates the pygame screen
	and allows for exiting of the pygame screen
'''

def Update():
	pygame.display.update()
	for e in pygame.event.get():
		if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
			sys.exit("Exiting")
		if e.type == MOUSEBUTTONDOWN:
		    print pygame.mouse.get_pos()[0], pygame.mouse.get_pos()[1],
		    return pygame.mouse.get_pos()

def load_polygons_here( fn="./info_gathering_paths.txt"):
    bdata = []
    for x in open( fn ):
        tmp = np.fromstring( x, dtype=float, sep=' ' )
        tmp = np.reshape( tmp/500, (-1,2) )
        tmp = np.vstack(( np.mean(tmp, axis=0, keepdims=True), tmp, tmp[0,:] ))
        bdata.append( tmp )
    

    return bdata

def add_isovist_obstacle(isovist, original_polygons):
	copy = original_polygons
	tmp = np.fromstring( isovist, dtype=float, sep=' ' )
	tmp = np.reshape( tmp/500, (-1,2) )
	tmp = np.vstack(( np.mean(tmp, axis=0, keepdims=True), tmp, tmp[0,:] ))
	copy.append( tmp )
	return copy

def load_polygons( fn="./info_gathering_paths.txt" ):
	polygonSegments = []
	for line in open( fn ):
		line = line.strip('\n')
		toList = line.split(' ')
		toList = [(float(x)/1000) for x in toList]
		
		it = iter(toList)
		toList = [toList[i:i+2] for i in range(0, len(toList), 2)]

		for pair in toList:
			#pair[1] = 1.0 - pair[1]
			pair[0] = int (pair[0] *1000)
			pair[1] = int (pair[1] *1000)

		#toList = [toList[i:i+2] for i in range(0, len(toList), 2)]
		#toList[-1].insert(0, toList[0][0])
		temp = []
		for i in xrange(1,len(toList)):
			pair = (toList[i-1], toList[i])
			temp.append(pair)
		temp.append((toList[0],toList[-1]))

		polygonSegments.append(temp)

	return polygonSegments

def IsUAVSeenByIntruder(uavLoc, radius, intended_path):
	for point in intended_path:
		distance = math.sqrt((uavLoc[0] - point[0]) ** 2 + (uavLoc[1] - point[1]) ** 2)
		if distance <= radius:
			return True, point
	return False, None

def IsIntruderSeenByUAV(RRTPath, intersections):

        # for point in RRTPath:
        #     isFound, loc = FindIntruderAtPoint(point, intersections)
        #     if isFound:
        #     	intLoc = [int(loc[0]), int(loc[1])]
        #         return True, intLoc # There's a bug ... breaks rrt

    for i in xrange(1,len(RRTPath)):
        segment = (RRTPath[i-1], RRTPath[i])

        for j in xrange(1,len(intersections)):
        	isovist_segment = (intersections[j-1], intersections[j])
        	intersect, param = GetIntersection(segment, isovist_segment)
        	
        	if intersect != None:
        		return True, RRTPath[i-1]
        #Check the losing of the polygon segment
        
        isovist_segment = (intersections[0], intersections[-1])
        intersect, param = GetIntersection(segment, isovist_segment)
        if intersect != None:
        	return True, intersections[0]

    return False, None

def FindIntruderAtPoint(pos, intersections):
    if intersections == []:
        return False
    points = intersections
    cn = 0  # the crossing number counter
    pts = points[:]
    pts.append(points[0])
    collision = None
    for i in range(len(pts) - 1):
        if (((pts[i][1] <= pos[1]) and (pts[i+1][1] > pos[1])) or ((pts[i][1] > pos[1]) and (pts[i+1][1] <= pos[1]))):
                if (pos[0] < pts[i][0] + float(pos[1] - pts[i][1]) / (pts[i+1][1] - pts[i][1]) * (pts[i+1][0] - pts[i][0])):
                        cn += 1
                        collision = pts[i]
    if bool(cn % 2)==1:
        return True, collision
    return False, None

def GetIntersection(ray, segment):
    # RAY in parametric: Point + Direction * T1
    r_px = ray[0][0]
    r_py = ray[0][1]

    # direction
    r_dx = ray[1][0] - ray[0][0]
    r_dy = ray[1][1] - ray[0][1]

    # SEGMENT in parametric: Point + Direction*T2
    s_px = segment[0][0]
    s_py = segment[0][1]

    # direction
    s_dx = segment[1][0] - segment[0][0]
    s_dy = segment[1][1] - segment[0][1]

    r_mag = math.sqrt(r_dx ** 2 + r_dy ** 2)
    s_mag = math.sqrt(s_dx ** 2 + s_dy ** 2)

    if r_mag == 0 or s_mag == 0:
    	return None, None
    # PARALLEL - no intersection
    if (r_dx/r_mag) == (s_dx/s_mag):
        if (r_dy/r_mag) == (s_dy/s_mag):
            return None, None
    
    denominator = float( -s_dx*r_dy + r_dx*s_dy )
    if denominator == 0:
        return None, None

    T1 = (-r_dy * (r_px - s_px) + r_dx * ( r_py - s_py)) / denominator
    T2 = (s_dx * ( r_py - s_py) - s_dy * ( r_px - s_px)) / denominator

    if T1 >= 0 and T1 <= 1 and T2 >= 0 and T2 <= 1:
        #Return the POINT OF INTERSECTION
        x = r_px+r_dx*T2
        y = r_py+r_dy*T2
        param = T2
        return ( x, y ), param

    return None, None


def GetReadablePath(path, screen):
	readable_path = []
	for i in xrange(1, len(path)):
		s_point = path[i-1]
		s_point = (int(s_point[0]*500), int(s_point[1]*500))

		e_point = path[i]
		e_point = (int(e_point[0]*500), int(e_point[1]*500))
		
		readable_path.append(s_point)
	return readable_path

def DrawMiniIsovistForPath(path, screen, goal, iso_radius = 55, 
							intersection = None, reroute_color=None, UAVSawInt = False):
	if intersection != None:
		pygame.draw.circle(screen, (0,255,255), intersection, iso_radius)
	# else:
	# 	for i in xrange(1, len(path), 5):
	# 		s_point = path[i-1]
	# 		pygame.draw.circle(screen, (0,255,255), s_point, size)
	caught_color = (255,0,0)
	plan_color = (0,0,255)
	color_in_use = plan_color
	if UAVSawInt:
		color_in_use = caught_color
	e_point = None
	for i in xrange(1, len(path)):
		s_point = path[i-1]
		e_point = path[i]

		if s_point == intersection:
			color_in_use = plan_color
		if reroute_color != None:
			color_in_use = reroute_color 
		pygame.draw.line(screen, color_in_use, s_point, e_point, 2)
	if e_point != None:
		pygame.draw.line(screen, color_in_use, e_point, goal, 2)
		

'''
	main function

'''

def main():
	ForceA = False
	ForceC = False
	UAVLoc2 = True
	ForceB = False

	if len(sys.argv) > 1 and sys.argv[1] == '1':
		UAVLoc2 = False
	if len(sys.argv) > 1 and sys.argv[1] == '2':
	 	UAVLoc2 = True

	if len(sys.argv) > 2 and sys.argv[2] == 'a':
		ForceA = True
	if len(sys.argv) > 2 and sys.argv[2] == 'c':
	 	ForceC = True
	if len(sys.argv) > 2 and sys.argv[2] == 'b':
		ForceB = True
	'''
	xdim and ydim of the pygame screen 
	'''
	xdim = 500
	ydim = 500
	backgroundFileName = "./cnts.png"
	background = pygame.image.load(backgroundFileName)
	background = pygame.transform.scale(background, (xdim, ydim))
	backgroundRect = background.get_rect()
	intruder_iso_radius = 70

	array = np.zeros([xdim, ydim])
	screen, clock = InitScreen(xdim, ydim)
	polygonSegments = load_polygons()

	# Clear canvas
	screen.fill((255,255,255))

	s = pygame.Surface((xdim,ydim))  	# the size of your rect
	s.set_alpha(0)                		# alpha level
	s.fill((255,255,255))           	# this fills the entire surface
	screen.blit(s, (0,0))

	start_paint = (int(101),int(455))
	end_paint = (int(336), int(47))

	start = np.atleast_2d( [(101 /500.0 ) ,(455/500.0 )] ) #(101, 455)
	end = np.atleast_2d( [(336/500.0 ),(47/500.0)] ) #(336, 47)
	original_polygons = load_polygons_here()
	X1, Y1, X2, Y2 = polygons_to_segments(original_polygons)

	if ForceA:
		block = "140 302 141 363 155 332"
		X1, Y1, X2, Y2 = polygons_to_segments(add_isovist_obstacle(block, original_polygons))
	# if ForceB:
	# 	block = "6 281 163 284 163 305 52 316 57 483 155 475 149 369 294 370 291 286 327 285 347 376 248 489 14 494 11 347"
	# 	X1, Y1, X2, Y2 = polygons_to_segments(add_isovist_obstacle(block, original_polygons))
	if ForceB:
		blockC= "287 297 289 372 311 329"
		blockA = "139 306 59 300 104 280"
		temp = add_isovist_obstacle(blockA, original_polygons)
		temp = add_isovist_obstacle(blockC, temp)
		X1, Y1, X2, Y2 = polygons_to_segments(temp)
	if ForceC:
		block = "65 307 304 305 172 255"
		X1, Y1, X2, Y2 = polygons_to_segments(add_isovist_obstacle(block, original_polygons))


	Update()

	isovist = iso.Isovist(polygonSegments)
	agentx = 262
	agenty = 214
	if UAVLoc2:
		agentx = 406
		agenty = 211
	UAVLocation = (agentx,agenty)
	mouseClick = None
	UAVForwardVector = (2, 121) # looking south
	intersections = [(262, 214), (236.0, 260), (286.0, 260)]
	isovist_block= "262 214 236 260 286 260"
	if UAVLoc2:
		intersections = [(406, 211), (384, 254), (430, 254)]
		isovist_block = "406 211 384 254 430 254"

	# Get intended path
	path = run_rrt( start, end, X1, Y1, X2, Y2)
	intended_path = GetReadablePath(path, screen)
	isIntruderFound, seen_loc = IsIntruderSeenByUAV(intended_path, intersections)
	isUAVFound, u_seen_loc= IsUAVSeenByIntruder(UAVLocation, intruder_iso_radius, intended_path)

	DrawMiniIsovistForPath(intended_path, screen, end_paint, iso_radius= intruder_iso_radius, intersection = u_seen_loc, UAVSawInt = isIntruderFound)

	intruderColor = (255,0,0)
	if not isIntruderFound:
		intruderColor = (0,255,0)

	# Draw Polygon for intersections (isovist)
	isovist_surface = pygame.Surface((xdim,ydim)) 
	isovist_surface.fill((255,255,255))
	isovist_surface.set_alpha(100)

	# JUST for drawing the isovist
	pygame.draw.polygon(isovist_surface, intruderColor, intersections)
	screen.blit(isovist_surface, isovist_surface.get_rect())

	pygame.draw.circle(screen, (100,100,100), UAVLocation, 5)
	pygame.draw.circle(screen, (0,255,0), start_paint, 7)
	pygame.draw.circle(screen, (255,0,0), end_paint, 7)
	

	# Draw segments
	for polygon in polygonSegments:
		for segment in polygon:
			pygame.draw.line(screen, (0, 0, 0), segment[0], segment[1] ,2)

	# Get re-routed path
	if isIntruderFound:
		X1, Y1, X2, Y2 = polygons_to_segments(add_isovist_obstacle(isovist_block, original_polygons))
		#X1, Y1, X2, Y2 = polygons_to_segments(original_polygons)
		seen_rrt = np.atleast_2d( [(seen_loc[0]/500.0 ),(seen_loc[1]/500.0)] )
		path = run_rrt( seen_rrt, end, X1, Y1, X2, Y2)
		rerouted_path = GetReadablePath(path, screen)

		DrawMiniIsovistForPath(rerouted_path, screen, end_paint, iso_radius= intruder_iso_radius, reroute_color=(0, 255, 0))

	if seen_loc != None:
		pygame.draw.circle(screen, (255,255,0), seen_loc, 5)
	if u_seen_loc != None:
		pygame.draw.circle(screen, (100,45,134), u_seen_loc, 5)


	#write legend
	pygame.draw.circle(screen, (100,100,100), [10,10], 5)
	pygame.draw.circle(screen, (0,255,0), [10,40], 7)
	pygame.draw.circle(screen, (255,0,0), [10,70], 7)
	pygame.draw.circle(screen, (255,255,0), [10,100], 5)
	pygame.draw.circle(screen, (100,45,134), [10,130], 5)

	myfont = pygame.font.SysFont("comicsansms", 20)

	# render text
	label = myfont.render("UAV", 2, (0,0,0))
	screen.blit(label, (30, 3))
	label = myfont.render("start", 2, (0,0,0))
	screen.blit(label, (30, 33))
	label = myfont.render("goal", 2, (0,0,0))
	screen.blit(label, (30, 63))
	label = myfont.render("detection of intruder by UAV", 2, (0,0,0))
	screen.blit(label, (30, 93))
	label = myfont.render("detection of UAV by intruder", 2, (0,0,0))
	screen.blit(label, (30, 123))

	label = myfont.render("Intruder detecter UAV = " + str(isUAVFound), 2, (0,0,0))
	screen.blit(label, (232, 400))
	label = myfont.render("UAV detected Intruder = " + str(isIntruderFound), 2, (0,0,0))
	screen.blit(label, (232, 440))

	pfont = pygame.font.SysFont("comicsansms", 40)
	label = pfont.render("A", 3, (150,150,150))
	screen.blit(label, (90, 278))
	label = pfont.render("B", 3, (150,150,150))
	screen.blit(label, (250, 278))
	label = pfont.render("C", 3, (150,150,150))
	screen.blit(label, (395, 278))

	while True:
		Update()
		pygame.time.delay(10)

		                    

if __name__ == '__main__':
    main()

