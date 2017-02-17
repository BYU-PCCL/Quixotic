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

def load_polygons_here( fn="./info_gathering_paths.txt" ):
    bdata = []
    count = 1
    for x in open( fn ):
    	if count != 0:
	        tmp = np.fromstring( x, dtype=float, sep=' ' )
	        tmp = np.reshape( tmp/500, (-1,2) )
	        tmp = np.vstack(( np.mean(tmp, axis=0, keepdims=True), tmp, tmp[0,:] ))
	        #tmp[:,1] = 1.0 - tmp[:,1]  # flip on the y axis
	        bdata.append( tmp )
       	count += 1
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

def IsIntruderSeen(RRTPath, intersections):

        for point in RRTPath:
            isFound = FindIntruderAtPoint(point, intersections)
            if isFound:
                return True

        for i in xrange(1,len(RRTPath)):
            segment = (RRTPath[i-1], RRTPath[i])

            for j in xrange(1,len(intersections)):
            	isovist_segment = (intersections[j-1], intersections[j])
            	intersect, param = GetIntersection(segment, isovist_segment)
            	if intersect != None:
            		return True
            #Check the losing of the polygon segment
            
            isovist_segment = (intersections[0], intersections[-1])
            intersect, param = GetIntersection(segment, isovist_segment)
            if intersect != None:
            	return True

        return False

def FindIntruderAtPoint(pos, intersections):
        if intersections == []:
            return False
        points = intersections
        cn = 0  # the crossing number counter
        pts = points[:]
        pts.append(points[0])
        for i in range(len(pts) - 1):
            if (((pts[i][1] <= pos[1]) and (pts[i+1][1] > pos[1])) or ((pts[i][1] > pos[1]) and (pts[i+1][1] <= pos[1]))):
                    if (pos[0] < pts[i][0] + float(pos[1] - pts[i][1]) / (pts[i+1][1] - pts[i][1]) * (pts[i+1][0] - pts[i][0])):
                            cn += 1
        if bool(cn % 2)==1:
            return True
        return False

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

	#101 455 336 47
	#### RRT STUFF
	start_paint = (int(101),int(455))
	end_paint = (int(336), int(47))

	start = np.atleast_2d( [(101 /500.0 ) ,(455/500.0 )] ) #(101, 455)
	end = np.atleast_2d( [(336/500.0 ),(47/500.0)] ) #(336, 47)
	X1, Y1, X2, Y2 = polygons_to_segments(load_polygons_here())

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

		pygame.draw.circle(screen, (0,255,0), start_paint, 7)
		pygame.draw.circle(screen, (255,0,0), end_paint, 7)
		
		#UAVForwardVector = direction
		UAVForwardVector = (2, 121)

		path = run_rrt( start, end, X1, Y1, X2, Y2)
		readable_path = []
		for i in xrange(1, len(path)):
			s_point = path[i-1]
			s_point = (int(s_point[0]*500), int(s_point[1]*500))

			e_point = path[i]
			e_point = (int(e_point[0]*500), int(e_point[1]*500))
			pygame.draw.line(screen, (0, 0, 255), s_point, e_point, 2)

			readable_path.append(s_point)
		pygame.draw.line(screen, (0, 0, 255), e_point, end_paint, 2)

		#isIntruderFound, intersections = isovist.IsIntruderSeen([mouse], UAVLocation, UAVForwardVector, UAVFieldOfVision = 45)
		
		intersections = [(262, 214), (236.0, 288), (236.0, 301),
		 (236.0, 305.0), (286.0, 305.0), 
		 (286.0, 301), (286.0, 276)]

	 	isIntruderFound = IsIntruderSeen(readable_path, intersections)
	 	#isIntruderFound = False
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
		#mouseClick = Update()
		Update()
			
		pygame.time.delay(10)

		                    

if __name__ == '__main__':
    main()

'''[(262, 214), (236.0, 288), (236.0, 301), (236.0, 305.0), (221, 360.0), (298, 360.0), (286.0, 305.0), (286.0, 301), (286.0, 276)]'''
