

import math
import random as rand
import numpy as np
import sys
import isovist as iso
from numpy import atleast_2d
import pickle
from my_rrt import *
from tqdm import tqdm

from multiprocessing import Pool

cand_start_locs = (
(0.044, 0.034) ,
(0.196, 0.032) ,
(0.752, 0.032) ,
(0.916, 0.048) ,
(0.104, 0.122) ,
(0.454, 0.116) ,
(0.776, 0.092) ,
(0.882, 0.19) ,
(0.614, 0.246) ,
(0.294, 0.376) ,
(0.09, 0.24) ,
(0.072, 0.454) ,
(0.24, 0.476) ,
(0.682, 0.428) ,
(0.844, 0.534) ,
(0.544, 0.614) ,
(0.252, 0.642) ,
(0.05, 0.702) ,
(0.032, 0.872) ,
(0.376, 0.77) ,
(0.542, 0.82) ,
(0.736, 0.762) ,
(0.896, 0.77) ,
(0.308, 0.972) ,
(0.684, 0.872) ,
(0.912, 0.946) )

def isovist_area(p):
    return 0.5 * abs(sum(x0*y1 - x1*y0 for ((x0, y0), (x1, y1)) in segments(p)))

def segments(p):
    return zip(p, p[1:] + [p[0]])

def single_smart_intruder_rrt_sarg( x ):
        # need to make sure that each process is in a different PRNG
        # state.  initialize seed using getpid(), but only do it once
        # per process (otherwise it will reset the next time the
        # worker is used!)
        if not hasattr( single_smart_intruder_rrt_sarg, '__my_init' ):
                single_smart_intruder_rrt_sarg.__my_init = True # use function attributes as static variables
                import os
                np.random.seed( os.getpid() )
        return single_smart_intruder_rrt( x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7] )

def single_smart_intruder_rrt(start, goal, X1, Y1, X2, Y2, isovist, X=8):
        rrt = run_rrt( rrt_loc(start), rrt_loc(goal), X1, Y1, X2, Y2)
        chosen_steps = rand.sample(range(1, len(rrt)), X)
        total_isovist_area = 0
        for j in chosen_steps:
                loc = rrt[j]
                prev = rrt[j-1]
                intersections = isovist.GetIsovistIntersections(loc, direction(loc, prev), full_iso=True)
                total_isovist_area += isovist_area(intersections) 
        return( rrt, total_isovist_area )

def smart_intruder_rrt_par(start, goal, X1, Y1, X2, Y2, isovist, N=30, X=8):
        areas = []
        rrts = []

        p = Pool( 12 )
        # we do all of this because Pool.map pickles its arguments, and you can't pickle a lambda...
        params = ((start, goal, X1, Y1, X2, Y2, isovist, X),) * N
        results = p.map( single_smart_intruder_rrt_sarg, params )

	for tmp_retval in results:
                rrts.append( tmp_retval[0] )
                areas.append( tmp_retval[1] )
	minindex = np.argmin(areas)
	return rrts[minindex]

# XXX deprecated in favor of the parallel version above
# def smart_intruder_rrt(start, goal, X1, Y1, X2, Y2, isovist, N=30, X=8):
# 	#GENERATE 30 RRTs:
#         areas = []
#         rrts = []
# 	for i in tqdm(xrange(N)):
#                 tmp_retval = single_smart_intruder_rrt(start, goal, X1, Y1, X2, Y2, isovist, X )
#                 rrts.append( tmp_retval[0] )
#                 areas.append( tmp_retval[1] )
# 	minindex = np.argmin(areas)
# 	return rrts[minindex]

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
	main function

'''

def roll_out( isovist, X1, Y1, X2, Y2 ):

	UAV_start = paint_loc( (0.968,0.032)) 
	UAV_end = paint_loc((0.016, 0.02)) 


        # SAMPLE INTRUDER
#	IntruderStart = paint_loc((0.06, 0.236))
#	IntruderGoal = paint_loc((.9,.9))

        # make sure the start and the goal are reasonably far apart
        while True:
                IntruderStart = paint_loc( cand_start_locs[ np.random.choice(len(cand_start_locs)) ] )
                IntruderGoal = paint_loc( cand_start_locs[ np.random.choice(len(cand_start_locs)) ] )
                if np.sum( (np.asarray(IntruderStart) - np.asarray(IntruderGoal))**2.0 ) >= 100000:
                        break

	# GET INTRUDER PATH AND DRAW INTENDED PATH
	#intruder_path = run_rrt( rrt_loc(IntruderStart), rrt_loc(IntruderGoal), X1, Y1, X2, Y2)
        intruder_path = smart_intruder_rrt_par( IntruderStart, IntruderGoal, X1, Y1, X2, Y2, isovist, N=10, X=8 )

	#THINK
	UAV_path = run_rrt( rrt_loc(UAV_start), rrt_loc(UAV_end), X1, Y1, X2, Y2)

	#STEPS THE INTRUDER TAKES TO GOAL
	e_point = None

	UAV_current_step = 1
	UAV_steps = 3

	UAV_curr_loc = UAV_start
	Intruder_curr_loc = IntruderStart
	for i in xrange(1, len(intruder_path)+1):
		#DRAW STEP FOR INTRUDER
		if i == len(intruder_path): #and e_point != None:
                        pass
		else:
			s = intruder_path[i-1]
			e = intruder_path[i]

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
                                pass
			else:
				s = UAV_path[j-1]
				e = UAV_path[j]
				
				# UAV CURRENT LOCATION
				UAV_curr_loc = paint_loc(e)
				if UAV_curr_loc == None:
					UAV_curr_loc = paint_loc(UAV_path[-1])

				if dist(Intruder_curr_loc, UAV_curr_loc) <= 70:
					fv = direction(paint_loc(e),paint_loc(s)) 
					if not(fv[0] == 0 or fv[1] == 0):
                                                try:
                                                        intersections = isovist.GetIsovistIntersections(UAV_curr_loc, fv)
                                                        intruder_seen = isovist.FindIntruderAtPoint(Intruder_curr_loc, intersections)
                                                except:
                                                        intruder_seen = 0 # XXX what should we really do?
						if intruder_seen:
                                                        return 1


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
			UAV_current_step = 1

        return 0

def main():

	polygonSegments = load_polygons()

	# isovist stuff
 	isovist = iso.Isovist( polygonSegments )

	X1, Y1, X2, Y2 = polygons_to_segments(load_polygons_here())

	
	INTRUDER_SEEN_COUNT = 0 

        for K in range( 100 ):

                result = roll_out( isovist, X1, Y1, X2, Y2 )
                INTRUDER_SEEN_COUNT += result

                print "%d/%d" % ( INTRUDER_SEEN_COUNT, K+1 )

if __name__ == '__main__':
    main()


