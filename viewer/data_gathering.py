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

def load_polygons_here( fn="./paths.txt" ):
    bdata = []
    for x in open( fn ):
        tmp = np.fromstring( x, dtype=float, sep=' ' )
        tmp = np.reshape( tmp/1000, (-1,2) )
        tmp = np.vstack(( np.mean(tmp, axis=0, keepdims=True), tmp, tmp[0,:] ))
        #tmp[:,1] = 1.0 - tmp[:,1]  # flip on the y axis
        bdata.append( tmp )
    return bdata

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


	'''border'''
	# polygonSegments.append([ 
	# 	[ (0,0),(1000,0) ], 
	# 	[ (1000,0),(1000,1000) ],
	# 	[ (1000,1000), (0,1000)],
	# 	[ (0,1000), (0,0) ]
	# 	])
        #print "toList:", toList
	# for p in polygonSegments:
	# 	print "\n", p
	return polygonSegments


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

	#screen.blit(background, backgroundRect)


	#### RRT STUFF
	# start_paint = (int(0.1 *500),int(0.1 *500))
	# end_paint = (int(0.9*500), int(0.9 *500))

	# start = np.atleast_2d( [(0.1 ) ,(0.1 )] )
	# end = np.atleast_2d( [(0.9 ),(0.9 )] )
	# X1, Y1, X2, Y2 = polygons_to_segments(load_polygons_here())

	# Draw segments
	for polygon in polygonSegments:
		for segment in polygon:
			pygame.draw.line(screen, (0, 0, 0), segment[0], segment[1] ,2)


	Update()

	isovist = iso.Isovist(polygonSegments)
	#(313, 115)
	agentx = 262
	agenty = 214
	UAVLocation = (agentx,agenty)
	mouseClick = None

	

	while True:
		if mouseClick != None:
			UAVLocation = mouseClick

		

		# Clear canvas
		screen.fill((255,255,255))
		s = pygame.Surface((xdim,ydim))  # the size of your rect
		s.set_alpha(0)                   # alpha level
		s.fill((255,255,255))            # this fills the entire surface
		screen.blit(s, (0,0))
		# Draw segments ( map )
		for polygon in polygonSegments:
			for segment in polygon:
				pygame.draw.line(screen, (0, 0, 0), segment[0], segment[1],2)


		mouse = pygame.mouse.get_pos()

		
		

		# getting directions
		dirx = mouse[0] - UAVLocation[0]
		diry = mouse[1] - UAVLocation[1]
		direction = (dirx, diry)

		#Draw hard coded RRT Path
		# pygame.draw.line(screen, (0, 0, 255), RRTPath[0], RRTPath[1],2)
		# for point in RRTPath:
		# 	pygame.draw.circle(screen, (255,100,255), point, 5)

		#pygame.draw.circle(screen, (255,255,255), start_paint, 15)
		#pygame.draw.circle(screen, (255,255,255), end_paint, 15)

		# pygame.draw.circle(screen, (0,255,0), start_paint, 10)
		# pygame.draw.circle(screen, (255,0,0), end_paint, 10)
		
		#UAVForwardVector = direction
		UAVForwardVector = (2, 121)
		#print UAVForwardVector

		#isIntruderFound, intersections = isovist.IsIntruderSeen([mouse], UAVLocation, UAVForwardVector, UAVFieldOfVision = 45)
		#print intersections
		isIntruderFound = False
		intersections = [(262, 214), (236.0, 288), (236.0, 301),
		 (236.0, 305.0), (286.0, 305.0), 
		 (286.0, 301), (286.0, 276)]
		intruderColor = (255,0,0)
		if isIntruderFound:
			intruderColor = (0,255,0)

		# Draw Polygon for intersections (isovist)
		isovist_surface = pygame.Surface((xdim,ydim)) 
		isovist_surface.set_alpha(80)

		# JUST for drawing the isovist
		if intersections != []:
			pygame.draw.polygon(isovist_surface, intruderColor, intersections)
			screen.blit(isovist_surface, isovist_surface.get_rect())

		pygame.draw.circle(screen, (100,100,100), UAVLocation, 5)
		#pygame.draw.circle(screen, (100,100,100), mouse, 5)

		#isox,isoy = UpdateMovement(isox,isoy)
		mouseClick = Update()
			
		pygame.time.delay(10)

		                    

if __name__ == '__main__':
    main()

'''[(262, 214), (236.0, 288), (236.0, 301), (236.0, 305.0), (221, 360.0), (298, 360.0), (286.0, 305.0), (286.0, 301), (286.0, 276)]'''
