import math
import pygame
from pygame.locals import *
import random as rand
import numpy as np
import sys
import isovist as iso
from numpy import atleast_2d
import pickle
from my_rrt import *
from tqdm import tqdm
#from new_intruder import *
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

	pygame.display.set_caption("Isovist")
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
		    return pygame.mouse.get_pos()

def load_polygons_here( fn="./paths.txt" ):
    bdata = []
    for x in open( fn ):
        tmp = np.fromstring( x, dtype=float, sep=' ' )
        tmp = np.reshape( tmp/1000, (-1,2) )
        tmp = np.vstack(( np.mean(tmp, axis=0, keepdims=True), tmp, tmp[0,:] ))
        #tmp[:,1] = 1.0 - tmp[:,1]  # flip on the y axis
        bdata.append( tmp )
    return bdata

def load_polygons( fn="./paths.txt" ):
	polygonSegments = []
	for line in open( fn ):
		line = line.strip('\n')
		toList = line.split(' ')
		toList = [(float(x)/1000) for x in toList]
		
		it = iter(toList)
		toList = [toList[i:i+2] for i in range(0, len(toList), 2)]

		for pair in toList:
			#pair[1] = 1.0 - pair[1]
			pair[0] = int (pair[0] *500)
			pair[1] = int (pair[1] *500)

		#toList = [toList[i:i+2] for i in range(0, len(toList), 2)]
		#toList[-1].insert(0, toList[0][0])
		temp = []
		for i in xrange(1,len(toList)):
			pair = (toList[i-1], toList[i])
			temp.append(pair)
		temp.append((toList[0],toList[-1]))
		
		polygonSegments.append(temp)

	dim = 500

	'''border'''
	polygonSegments.append([ 
		[ (-5,-5),(505,-5) ], 
		[ (505,-5),(505,505) ],
		[ (505,505), (-5,505)],
		[ (-5,505), (-5,-5) ]
		])
        #print "toList:", toList
	# for p in polygonSegments:
	# 	print "\n", p
	return polygonSegments



def DrawRRT(path, screen, goal, printPoints=False, highlight = None):
	color_in_use = (255,255,0)
	if highlight != None:
		color_in_use = highlight
	e_point = None

	for i in xrange(1, len(path)):
		s = path[i-1]
		e = path[i]
		s_point = (int(s[0]*500), int(s[1]*500))
		e_point = (int(e[0]*500), int(e[1]*500))
		pygame.draw.line(screen, color_in_use, s_point, e_point, 1)
	if e_point != None:
		pygame.draw.line(screen, color_in_use, e_point, goal, 1)
	
	# skip = 8

	# for i in xrange(skip-1, len(path), skip):
	# 	s = path[i-skip+1]
	# 	e = path[i]
	# 	s_point = (int(s[0]*500), int(s[1]*500))
	# 	e_point = (int(e[0]*500), int(e[1]*500))
	# 	pygame.draw.line(screen, (255,0,0), s_point, e_point, 1)
	# if e_point != None:
	# 	pygame.draw.line(screen, (255,0,0), e_point, goal, 1)

def load_data():
    try:
        with open("rrt_paths-new.dat") as f:
            x = pickle.load(f)
    except:
        x = []
    return x

def save_data(data):
    with open("rrt_paths-new.dat", "wb") as f:
        pickle.dump(data, f)


def paint_loc(point):
	return (int(point[0] * 500), int(point[1] * 500))

def rrt_loc(point):
	loc = np.atleast_2d( [( point[0]/500.0) ,( point[1]/500.0 )] )
	return  loc

def go_south(point, amt = 1):
	return (point[0], point[1]+ amt)

def dist(one, two):
	xs = one[0] - two[0]
	ys = one[1] - two[1]
	return math.sqrt(xs**2 + ys**2)
def direction (now, before):
	return (now[0]-before[0], now[1] - before[1])

'''
# choose path that has chooses least total isovist area (360 degree isovist)
smart_intruder_rrt(start_loc, goal_loc):
	for n generated rrts:
		total_isovist_area[n] = get total isovist area every x steps 
	return rrt with min total_isovist_area[n]

'''
def isovist_area(p):
    return 0.5 * abs(sum(x0*y1 - x1*y0 for ((x0, y0), (x1, y1)) in segments(p)))

def segments(p):
    return zip(p, p[1:] + [p[0]])

def smart_intruder_rrt(start, goal, X1, Y1, X2, Y2, isovist, N=30, X=8):
	#GENERATE 30 RRTs:
	rrts = []
	areas = []
	for i in tqdm(xrange(N)):
		rrts.append(run_rrt( rrt_loc(start), rrt_loc(goal), X1, Y1, X2, Y2))
		chosen_steps = rand.sample(range(1, len(rrts[i])), X)
		total_isovist_area = 0
		for j in chosen_steps:
			loc = rrts[i][j]
			prev = rrts[i][j-1]
			intersections = isovist.GetIsovistIntersections(loc, direction(loc, prev), full_iso=True)
			total_isovist_area += isovist_area(intersections) 
		areas.append(total_isovist_area)
	minindex = np.argmin(areas)
	return rrts[minindex]



'''
	main function

'''

def main():

	

	'''
	xdim and ydim of the pygame screen 
	'''
	xdim = 500
	ydim = 500
	backgroundFileName = "./cnts.png"
	background = pygame.image.load(backgroundFileName)
	background = pygame.transform.scale(background, (xdim, ydim))
	backgroundRect = background.get_rect()

	array = np.zeros([xdim, ydim])
	screen, clock = InitScreen(xdim, ydim)
	polygonSegments = load_polygons()

	# isovist stuff
 	isovist = iso.Isovist( polygonSegments )


	# Clear canvas
	screen.fill((255,255,255))

	s = pygame.Surface((xdim,ydim))  	# the size of your rect
	s.set_alpha(0)                		# alpha level
	s.fill((255,255,255))           	# this fills the entire surface
	screen.blit(s, (0,0))

	screen.blit(background, backgroundRect)


	X1, Y1, X2, Y2 = polygons_to_segments(load_polygons_here())

	
	for polygon in polygonSegments:
		for segment in polygon:
			pygame.draw.line(screen, (225, 225, 225), segment[0], segment[1],1)
	
	Update()

	INTRUDER_SEEN_COUNT = 0 

	UAV_start = paint_loc( (0.968,0.032)) 
	UAV_end = paint_loc((0.016, 0.02)) 

	#==== testing full isovist area
	# test_start = paint_loc((0.53, 0.28))
	# pygame.draw.circle(screen, (255,255,0), test_start, 7)
	# isovist_surface = pygame.Surface((xdim,ydim)) 
	# isovist_surface.set_alpha(50)
	# intersections = isovist.GetIsovistIntersections(test_start, (0,10), full_iso=True)
	# if intersections != []:
	# 	pygame.draw.polygon(isovist_surface, (255,255,0), intersections)
	# 	screen.blit(isovist_surface, isovist_surface.get_rect())
	# 	print isovist_area(intersections)
	# Update()
	# print "Paused"
	# raw_input()
	#======== 


	K = 0
	while K <= 0:

		# SAMPLE INTRUDER
		IntruderStart = paint_loc((0.06, 0.236))
		IntruderGoal = paint_loc((.9,.9))

		pygame.draw.circle(screen, (0,255,0), IntruderStart, 5)
		pygame.draw.circle(screen, (255,0,0), IntruderGoal, 5)

		# GET INTRUDER PATH AND DRAW INTENDED PATH
		# intruder_path = run_rrt( rrt_loc(IntruderStart), rrt_loc(IntruderGoal), X1, Y1, X2, Y2)
		# DrawRRT(intruder_path, screen, IntruderGoal)
		intruder_path = smart_intruder_rrt(IntruderStart, IntruderGoal, X1, Y1, X2, Y2, isovist, N=10, X=8)

		#THINK
		UAV_path = run_rrt( rrt_loc(UAV_start), rrt_loc(UAV_end), X1, Y1, X2, Y2)
		DrawRRT(UAV_path, screen, UAV_end, highlight = (0,255,0))

		#STEPS THE INTRUDER TAKES TO GOAL
		e_point = None
		intruder_walking_color = (255, 0, 0)
		UAV_searching_color = (255, 0, 255)

		UAV_current_step = 1
		UAV_steps = 3

		UAV_curr_loc = UAV_start
		Intruder_curr_loc = IntruderStart
		for i in xrange(1, len(intruder_path)+1):
			#DRAW STEP FOR INTRUDER
			if i == len(intruder_path): #and e_point != None:
				pygame.draw.line(screen, intruder_walking_color, paint_loc(intruder_path[-1]), IntruderGoal, 1)
			else:
				s = intruder_path[i-1]
				e = intruder_path[i]
				pygame.draw.line(screen, intruder_walking_color, paint_loc(s), paint_loc(e), 1)

			#INTRUDER CURRENT LOCATION
			Intruder_curr_loc = paint_loc(e)
			if Intruder_curr_loc == None:
				Intruder_curr_loc = paint_loc(intruder_path[-1])
			

			
			
			#FOR EVERY 1 STEP OF INTRUDER,  N STEPS FOR UAV
			stepsToGo = UAV_current_step+UAV_steps
			if len(UAV_path) - UAV_current_step < UAV_steps:
				stepsToGo = len(UAV_path) + 1
				
			for j in xrange(UAV_current_step, stepsToGo):
				if j == len(UAV_path): #and e_point != None:
					pygame.draw.line(screen, UAV_searching_color, paint_loc(UAV_path[-1]), UAV_end, 1)
				else:
					s = UAV_path[j-1]
					e = UAV_path[j]
					pygame.draw.line(screen, UAV_searching_color, paint_loc(s), paint_loc(e), 1)
					
					# UAV CURRENT LOCATION
					UAV_curr_loc = paint_loc(e)
					if UAV_curr_loc == None:
						UAV_curr_loc = paint_loc(UAV_path[-1])

					if dist(Intruder_curr_loc, UAV_curr_loc) <= 70:
						fv = direction(paint_loc(e),paint_loc(s)) 
						if not(fv[0] == 0 or fv[1] == 0):
							intersections = isovist.GetIsovistIntersections(UAV_curr_loc, fv)
							intruder_seen = isovist.FindIntruderAtPoint(Intruder_curr_loc, intersections)
							if intruder_seen:
								INTRUDER_SEEN_COUNT += 1
								print "INTRUDER SEEN!"
								#raw_input()


				Update()
				pygame.time.delay(10)
			UAV_current_step += UAV_steps
			if stepsToGo >= len(UAV_path):
				south = 50
				#THINK AGAIN!
				UAV_start = go_south(UAV_start, amt=south)
				UAV_end = go_south(UAV_end, amt=south)
				
				if UAV_start[1] >= 500:
					UAV_start = paint_loc( (0.968,0.032)) 
					UAV_end = paint_loc((0.016, 0.02)) 

				temp = UAV_start
				UAV_start = UAV_end
				UAV_end = temp
				
				UAV_path = run_rrt( rrt_loc(UAV_start), rrt_loc(UAV_end), X1, Y1, X2, Y2)
				DrawRRT(UAV_path, screen, UAV_end, highlight = (0,255,0))
				UAV_current_step = 1
			

			Update()
			pygame.time.delay(10)

		K += 1


	

	# index = getPathIndex()
	# print index
	
	myfont = pygame.font.SysFont("arial", 20)
	while True:
		mouseClick = Update()
		if mouseClick != None:
			loc =  mouseClick[0]/500.0, mouseClick[1]/500.0
			print loc,","
			pygame.draw.circle(screen, (255,255,0), mouseClick, 7)
			label = myfont.render(str(loc), 3, (255,255,0))
			label_loc = (mouseClick[0] -30, mouseClick[1] - 20)
			screen.blit(label,label_loc)
			
		pygame.time.delay(10)

		                    

if __name__ == '__main__':
    main()


