
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 
import q
import erps

import math
import random as rand
import numpy as np
import isovist as iso
from numpy import atleast_2d
from my_rrt import *
from tqdm import tqdm

from multiprocessing import Pool

from path_kde import *

a2d = atleast_2d

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

#
# ==========================================================================
#

def memoize(f):
        """ Memoization decorator for functions taking one or more arguments. """
        class memodict(dict):
                def __init__(self, f):
                        self.f = f
                def __call__(self, *args):

                        key = str([str(x) for x in args ])
                        if self.has_key( key ):
                                return self[ key ]
                        else:
                                rval = self.f( *args )
                                self[key] = rval
                                return rval
                        # return self[args] # XXX more elegant, but numpy arrays are unhashable
#                def __missing__(self, key):
#                        ret = self[key] = self.f( *args )
#                        return ret

        return memodict(f)

#
# ==========================================================================
#

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
        rrt = run_rrt( a2d(rrt_loc(start)), a2d(rrt_loc(goal)), X1, Y1, X2, Y2)

        try:
                chosen_steps = rand.sample( range(1, len(rrt)), np.minimum(X,len(rrt)-2) )
        except:
                return( rrt, 0 )

        total_isovist_area = 0
        for j in chosen_steps:
                loc = rrt[j]
                prev = rrt[j-1]
                intersections = isovist.GetIsovistIntersections(loc, direction(loc, prev), full_iso=True)
                total_isovist_area += isovist_area(intersections) 
        return( rrt, total_isovist_area )

# XXX hacked out the kwargs because they're not memoize compatible...
@memoize
def smart_intruder_rrt_par(start, goal, X1, Y1, X2, Y2, isovist, N, X):
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
def smart_intruder_rrt(start, goal, X1, Y1, X2, Y2, isovist, N=30, X=8):
	#GENERATE 30 RRTs:
        areas = []
        rrts = []
	for i in tqdm(xrange(N)):
                tmp_retval = single_smart_intruder_rrt(start, goal, X1, Y1, X2, Y2, isovist, X )
                rrts.append( tmp_retval[0] )
                areas.append( tmp_retval[1] )
	minindex = np.argmin(areas)
	return rrts[minindex]

#
# ==========================================================================
#

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

#
# ==========================================================================
#

# XXX this has nothing to do with that awful 1992 movie
class Coolworld( object ):
        def __init__( self ):
                self.cur_t = 0
                self.intersections = []

        def run( self, Q ):

                cnt = len(cand_start_locs)

                sind = Q.choice(p=1.0/cnt*np.ones((1,cnt)),name="is")
                gind = Q.choice(p=1.0/cnt*np.ones((1,cnt)),name="ig")

                IntruderStart = paint_loc( cand_start_locs[ sind ] )
                IntruderGoal = paint_loc( cand_start_locs[ gind ] )

                intruder_path = smart_intruder_rrt_par( IntruderStart, IntruderGoal,
                                                    self.X1, self.Y1, self.X2, self.Y2,
                                                    self.isovist, 10, 8 )

                pvals = np.zeros((1,self.cur_t+1))
#                for t, pt in enumerate( intruder_path[0:self.cur_t+1] ):
                for t, pt in enumerate( intruder_path[0:self.cur_t] ): # XXX hack.  fixme???
                        try:
                                intruder_seen = isovist.FindIntruderAtPoint( paint_loc(pt), self.intersections[t] )
                        except:
                                intruder_seen = False  # XXX probably self.intersections[t] == None
                        pvals[:,t:t+1] = 0.999*intruder_seen + 0.001*(1-intruder_seen)

#                print pvals
                data = Q.flip( p=pvals, name="data" )

                return IntruderStart, IntruderGoal, intruder_path, pvals, sind, gind

#
# ==========================================================================
#

def uav_plan( t, results, UAV_loc, X1, Y1, X2, Y2 ):

        # infer s,g,rrt | data
        print "    inferring..."
        global model
        Q = q.Q( model )
        # we haven't seen the intruder up to this point.  if we had,
        # the episode would have ended.
        Q.condition( name="data", value=a2d([False]*(t+1)) )
        model.cur_t = t
        model.intersections = results['intersections']
        Q.analyze()
        results, scores = Q.opt_adam( alpha=0.01, itercnt=10, rolloutcnt=10 )

        # sample a bunch of RRTs from the variational distribution
        print "    sampling..."
        set_of_rrts = []
        q_samples = []
        for i in range( 10 ):
                print "      sample %d" % i
                rval = Q.run_model()
                set_of_rrts.append( rval[2] ) # start_loc, goal_loc, rrt_path
                q_samples.append( rval )

        # construct a heat cube
        print "    planning..."
        pts = []
        for k in set_of_rrts:
                if t < len( k ):
                        pts.append( k[t] )
                heatmap = make_heatmap( pts, ss=100 )

        # plan to go to the region of highest probability
        goal_loc = np.unravel_index( np.argmax( heatmap ), heatmap.shape ) # XXX row/column madness?
        uav_plan_t = run_rrt( a2d(UAV_loc), a2d(rrt_loc(goal_loc)), X1, Y1, X2, Y2 )

        return uav_plan_t, q_samples, Q

def uav_plan_simple( t, results, UAV_loc, X1, Y1, X2, Y2 ):
        uav_plan_t = run_rrt( a2d(UAV_loc), a2d((0.1,0.1)), X1, Y1, X2, Y2 )        
        return uav_plan_t, [], None

#
# ==========================================================================
#

def roll_out( k, isovist, X1, Y1, X2, Y2 ):

        UAV_steps = 3
        Int_steps = 1

        # SAMPLE UAV
	UAV_start = paint_loc( cand_start_locs[ np.random.choice(len(cand_start_locs)) ] )

        # SAMPLE INTRUDER
        # make sure the start and the goal are reasonably far apart
        while True:
                IntruderStart = paint_loc( cand_start_locs[ np.random.choice(len(cand_start_locs)) ] )
                IntruderGoal = paint_loc( cand_start_locs[ np.random.choice(len(cand_start_locs)) ] )
                if np.sum( (np.asarray(IntruderStart) - np.asarray(IntruderGoal))**2.0 ) >= 100000:
                        break

        print IntruderStart
        print IntruderGoal

	# GET TRUE INTRUDER PATH
        print "intruder planning..."
        intruder_path = smart_intruder_rrt_par( IntruderStart, IntruderGoal, X1, Y1, X2, Y2, isovist, 10, 8 )
        # print intruder_path

        results = {}
        results['intruder_path'] = intruder_path
        results['intruder_start'] = IntruderStart
        results['intruder_goal'] = IntruderGoal
        results['intruder_seen'] = []
        results['uav_start'] = UAV_start
        results['uav_plans'] = []
        results['intersections'] = []
        results['uav_locs'] = []
        results['int_locs'] = []
        results['q_samples'] = []
#        results['q_s'] = []

        UAV_loc = rrt_loc( UAV_start )
        Int_loc = rrt_loc( IntruderStart )

        # simulate
        intruder_seen = False
        for t in range(200):
                print "  timestep %d" % t
                if t >= len( intruder_path ):
                        print "  No detection!"
                        break

                # update UAV plan
                uav_plan_t, q_samples, Q = uav_plan( t, results, UAV_loc, X1, Y1, X2, Y2 )
#                uav_plan_t, q_samples, Q = uav_plan_simple( t, results, UAV_loc, X1, Y1, X2, Y2 )
#                print uav_plan_t

                # step UAV and intruder
                Int_loc = intruder_path[t]
                if len(uav_plan_t) > UAV_steps:
                        UAV_loc = uav_plan_t[ UAV_steps-1 ]
                        fv = direction( paint_loc( uav_plan_t[UAV_steps-1] ),
                                        paint_loc( uav_plan_t[UAV_steps-2] ) ) 
                else:
#                        print "  UAV plan too short!"
                        fv = (0,0)

                intersections = None
                if not( fv[0] == 0 or fv[1] == 0 ):
                        try:
                                intersections = isovist.GetIsovistIntersections( paint_loc(UAV_loc), fv)
                                intruder_seen = isovist.FindIntruderAtPoint( paint_loc(Int_loc), intersections )
                        except:
                                intersections = None
                                intruder_seen = 0 # XXX what should we really do?

                results['q_samples'].append( q_samples )
#                results['q_s'].append( Q )
                results['intruder_seen'].append( intruder_seen )
                results['uav_plans'].append( uav_plan_t )
                results['intersections'].append( intersections )
                results['uav_locs'].append( UAV_loc )
                results['int_locs'].append( Int_loc )

                np.save( './results_%d_%d.npy' % (k,t), results )

                if intruder_seen:
                        print "  Detection!"
                        print UAV_loc
                        print Int_loc
                        break

        return results, intruder_seen

#
# ==========================================================================
# MAIN
# ==========================================================================
#

polygonSegments = load_polygons()
isovist = iso.Isovist( polygonSegments )
X1, Y1, X2, Y2 = polygons_to_segments(load_polygons_here())

model = Coolworld()
model.isovist = isovist
model.X1 = X1
model.Y1 = Y1
model.X2 = X2
model.Y2 = Y2

results = []
#for K in tqdm(xrange( 100 )):
for K in xrange( 100 ):
        print "============================================="
        print "ITERATION %d" % K

#        rval = roll_out( K, isovist, X1, Y1, X2, Y2 )
#        results.append( rval )

        try: 
                rval = roll_out( K, isovist, X1, Y1, X2, Y2 )
                results.append( rval )
        except Exception as e:
                print "  XXX something went wrong..."
                print e
                print e.args
                print e.message
