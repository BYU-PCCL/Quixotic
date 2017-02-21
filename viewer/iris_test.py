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

def GetReadablePath(path):
	readable_path = []
	for i in xrange(1, len(path)):
		s_point = path[i-1]
		s_point = (int(s_point[0]*500), int(s_point[1]*500))
		
		e_point = path[i]
		e_point = (int(e_point[0]*500), int(e_point[1]*500))
		pygame.draw.line(screen, (225, 225, 0), s_point, e_point, 1)
		readable_path.append(s_point)
	return readable_path

def DrawRRT(path, screen, goal, printPoints=False, highlight = None):
	color_in_use = (0,255,255)
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

	# Clear canvas
	screen.fill((255,255,255))

	s = pygame.Surface((xdim,ydim))  	# the size of your rect
	s.set_alpha(0)                		# alpha level
	s.fill((255,255,255))           	# this fills the entire surface
	screen.blit(s, (0,0))

	screen.blit(background, backgroundRect)


	#### RRT STUFF
	start_paint = (int(0.1 *500),int(0.1 *500))
	end_paint = (int(0.9*500), int(0.9 *500))

	start = np.atleast_2d( [(0.1 ) ,(0.1 )] )
	end = np.atleast_2d( [(0.9 ),(0.9 )] )
	X1, Y1, X2, Y2 = polygons_to_segments(load_polygons_here())

	# Draw segments
	for polygon in polygonSegments:
		for segment in polygon:
			pygame.draw.line(screen, (225, 225, 225), segment[0], segment[1] ,1)


	Update()

	
	screen.blit(background, backgroundRect)
	for polygon in polygonSegments:
		for segment in polygon:
			pygame.draw.line(screen, (225, 225, 225), segment[0], segment[1],1)
	
	numPaths = 30
	#numPaths = 1
	inbetweenPoints = [ (0.674, 0.102) ,
						(0.554, 0.264) ,
						(0.288, 0.658) ,
						(0.18, 0.696) ,
						(0.286, 0.848) ,
						(0.502, 0.804) ,
						(0.848, 0.444) ,
						(0.616, 0.472) ,
						(0.41, 0.566) ,
						(0.744, 0.688)]

	cand_uav_locs = [(0.532, 0.28) ,
				(0.66, 0.472) ,
				(0.242, 0.484) ,
				(0.294, 0.666) ,
				(0.458, 0.752) ,
				(0.584, 0.64) ,
				(0.55, 0.1) ,
				(0.364, 0.194) ,
				(0.47, 0.466) ,
				(0.742, 0.682) ,]

	for loc in cand_uav_locs:
		pygame.draw.circle(screen, (255,255,0), (int(loc[0]*500), int(loc[1]*500)), 7)

	rrt_paths = load_data()	
	paths = []				
	for i in xrange(numPaths):
		# inbetween = np.atleast_2d([inbetweenPoints[i%10]])
		# path = run_rrt( start, inbetween, X1, Y1, X2, Y2)
		# path2 = run_rrt( inbetween, end, X1, Y1, X2, Y2)
		# path.extend(path2)
		# paths.append(path)

		path = rrt_paths[i]
		if i == 1:
			DrawRRT(path, screen, end_paint, printPoints = True, highlight=(0,255,0))
		elif i == 27:
			DrawRRT(path, screen, end_paint, printPoints = True, highlight=(255,0,0))
		else:
			DrawRRT(path, screen, end_paint, printPoints = True)
		Update()
	
		#save_data(paths)
	pygame.draw.circle(screen, (0,255,0), start_paint, 5)
	pygame.draw.circle(screen, (255,0,0), end_paint, 5)

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


