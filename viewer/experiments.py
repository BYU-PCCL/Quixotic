import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 

import q
import isovist as iso
import math
from my_rrt import *

from numpy import atleast_2d
a2d = atleast_2d


class Experiments():

	def __init__( self ):
		self.X1, self.Y1, self.X2, self.Y2 = polygons_to_segments( self.load_polygons( "./paths.txt" ) )
		self.isovist = iso.Isovist([self.listify_segs(self.X1, self.Y1, self.X2, self.Y2)])

		self.intruder_start_goals = [((0.114, 0.362) ,(0.55, 0.848)) ,
		((0.378, 0.128) ,(0.824, 0.562)) ,
		((0.292, 0.034) ,(0.284, 0.842)) ,
		((0.71, 0.202) ,(0.37, 0.864)) ,
		((0.91, 0.832) ,(0.146, 0.572)), 
		((0.854, 0.598) ,(0.29, 0.076)) ,
		((0.828, 0.408) ,(0.12, 0.41)) ,
		((0.614, 0.882) ,(0.368, 0.152)) ,
		((0.154, 0.608) ,(0.736, 0.208)) ,
		((0.668, 0.934) ,(0.742, 0.14)) ,
		((0.12, 0.504)  ,(0.714, 0.018)) ,
		((0.722, 0.808) ,(0.29, 0.14) ),
		((0.854, 0.424) ,(0.19, 0.68)) ,
		((0.802, 0.152) ,(0.116, 0.456)) ,
		((0.168, 0.158) ,(0.886, 0.736)) ,
		((0.802, 0.692) ,(0.166, 0.456)) ,
		((0.764, 0.246) ,(0.572, 0.866)) ,
		((0.15, 0.69) ,(0.856, 0.46)) ,
		((0.904, 0.864) ,(0.084, 0.502)) ,
		((0.782, 0.274) ,(0.644, 0.88)) ,
		((0.15, 0.184) ,(0.802, 0.3)) ,
		((0.156, 0.204) ,(0.854, 0.516)),
		((0.968,0.032) ,(.9,.9)) ]

	def listify_segs(self,rx1,ry1,rx2,ry2 ):
		result = []
		for i in range(rx1.shape[0]):
		    result.append( [ (rx1[i],ry1[i]), (rx2[i],ry2[i]) ] )
		return result

	def load_polygons(self, fn="./paths.txt" ):
		bdata = []
		for x in open( fn ):
		    tmp = np.fromstring( x, dtype=float, sep=' ' )
		    tmp = np.reshape( tmp/1000, (-1,2) )
		    tmp = np.vstack(( np.mean(tmp, axis=0, keepdims=True), tmp, tmp[0,:] ))
		    #tmp[:,1] = 1.0 - tmp[:,1]  # flip on the y axis
		    bdata.append( tmp )
		return bdata

	def rrt_loc(self, point):
		loc = np.atleast_2d( [( point[0]) ,( point[1])] )
		return  loc

	def sample_uav_start(self, K):
		return (0.968,0.032) 

	def sample_uav_goal(self, K):
		return (0.016, 0.02)

	def sample_intr_start_goal(self, K):
		return self.intruder_start_goals[K]
		#return (0.06, 0.236)

	def dist(self, one, two):
		xs = one[0] - two[0]
		ys = one[1] - two[1]
		return math.sqrt(xs**2 + ys**2)

	def direction (self, now, before):
		return (now[0]-before[0], now[1] - before[1])
		#return ((now[0]-before[0])*500, (now[1] - before[1]*500))

	def step_south(self, point, amt = 1):
		return (point[0], point[1]+ amt)

	def run_naive(self):
		
		INTRUDER_SEEN_COUNT = 0 
		#K is 1 right now.. can be up to 20some 
		for K in xrange(1):
			#SAMPLE INTRUDER START AND GOAL LOCS
			path_info = self.sample_intr_start_goal(K)
			intruder_start = path_info[0]
			intruder_goal = path_info[1]
			intruder_curr_loc = intruder_start
			#GET INTRUDER'S PATH
			intruder_path = run_rrt( self.rrt_loc(intruder_start), 
				self.rrt_loc(intruder_goal), self.X1, self.Y1, self.X2, self.Y2)
			
			#VARIABLES
			UAV_j = 1 #future index in rrt path
			UAV_steps_faster = 5 #how many more steps uav takes
			intr_i = 0 #index in rrt path
			#EPISODE
			while True:
				#=========================================
				#INTRUDER CURRENT LOCATION
				intruder_curr_loc = intruder_path[intr_i]
				#=========================================
				#=========================================
				#GET UAV SAMPLE START AND GOAL LOCS
				UAV_start = self.sample_uav_start(K)
				UAV_goal = self.sample_uav_goal(K)
				UAV_curr_loc = UAV_start
				#UAV THINK (NAIVE) PLAN PATH
				UAV_path = run_rrt( self.rrt_loc(UAV_start), 
					self.rrt_loc(UAV_goal), self.X1, self.Y1, self.X2, self.Y2)
				#=========================================

				#=========================================
				#FOR EVERY 1 STEP OF INTRUDER,  UAV TAKES 5 STEPS
				detected = False
				stride = UAV_j + UAV_steps_faster
				if len(UAV_path) - UAV_j < UAV_steps_faster:
					stride = len(UAV_path)
				for j in xrange(UAV_j, stride):
					UAV_curr_loc = UAV_path[j]
					if self.dist(intruder_curr_loc, UAV_curr_loc) <= .14:
						fv = self.direction(UAV_curr_loc,UAV_path[j-1]) 
						if not(fv[0] == 0 or fv[1] == 0):
							intersections = self.isovist.GetIsovistIntersections(UAV_curr_loc, fv)
							intruder_seen = self.isovist.FindIntruderAtPoint(Intruder_curr_loc, intruder_curr_loc)
							if intruder_seen:
								INTRUDER_SEEN_COUNT += 1
								detected = True
								print "INTRUDER SEEN!", INTRUDER_SEEN_COUNT
								break
				UAV_j += UAV_steps_faster
				intr_i += 1
				#=========================================
				if intr_i == len(intruder_path):
					print "RESAMPLING FOR NEW EPISODE: INTRUDER NOT DETECTED"
					break
					
				if detected:
					print "RESAMPLING FOR NEW EPISODE: INTRUDER DETECTED"
					break
					
				if stride >= len(UAV_path):
					print "REPLAN: INTRUDER NOT DETECTED"
					step_south = 50
					UAV_start = self.step_south(UAV_start, amt=step_south)
					UAV_goal= self.step_south(UAV_goal, amt=step_south)
					if UAV_start[1] >= 500:
						UAV_start = self.sample_uav_start(K)
						UAV_goal = self.sample_uav_goal(K) 
					temp = UAV_start
					UAV_start = UAV_goal
					UAV_goal = temp
					#UAV THINK (NAIVE) PLAN PATH
					UAV_path = run_rrt( self.rrt_loc(UAV_start), 
						self.rrt_loc(UAV_goal), self.X1, self.Y1, self.X2, self.Y2)
					UAV_j = 1
				#=========================================

def main():
	e = Experiments()
	#print e.sample_intr_goal()
	e.run_naive()


if __name__ == '__main__':
    main()

