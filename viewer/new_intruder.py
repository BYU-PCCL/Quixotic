import math
import pygame
from pygame.locals import *
import random as rand
import numpy as np
import sys
import isovist as iso
from numpy import atleast_2d
from tqdm import tqdm
import pickle

from my_rrt import *

def listify_segs(rx1,ry1,rx2,ry2 ):
    result = []
    for i in range(rx1.shape[0]):
        result.append( [ (rx1[i],ry1[i]), (rx2[i],ry2[i]) ] )
    return result

def load_data():
    try:
        with open("rrt_paths-new.dat") as f:
            x = pickle.load(f)
    except:
        x = []
    return x

X1, Y1, X2, Y2 = polygons_to_segments(load_polygons())
isovist = iso.Isovist( [listify_segs(X1, Y1, X2, Y2)] )



#===================GIVEN DATA==========================


#candidate UAV locations (10)
cand_uav_locs = [(0.532, 0.28) ,
				(0.66, 0.472) ,
				(0.242, 0.484) ,
				(0.294, 0.666) ,
				(0.458, 0.752) ,
				(0.584, 0.64) ,
				(0.55, 0.1) ,
				(0.364, 0.194) ,
				(0.47, 0.466) ,
				(0.742, 0.682) ]

w = [0.08, 0.18, 0.10,  0.10, 0.04,
	 0.13, 0.08, 0.14, 0.10, 0.05]



#set start and goal locations
start = np.atleast_2d( [(0.1 ) ,(0.1 )] )
end = np.atleast_2d( [(0.9 ),(0.9 )] )

rrt_paths = load_data()

num_paths = len(rrt_paths)
num_locs = len(cand_uav_locs)

UAVForwardVector = (2, 121) # looking south

#=======================================================

def getPathIndex():
	scores = []
	for i in tqdm(xrange(num_paths)):
		path = rrt_paths[i]
		detections = []
		for j in xrange(num_locs):
			uav_loc = cand_uav_locs[j]
			detected, intersections = isovist.IsIntruderSeen(path, uav_loc, UAVForwardVector, UAVFieldOfVision = 45)
			detections.append(detected)
		#print detections
		score_i = np.sum(np.multiply(w,detections))
		scores.append(score_i)

	#print scores
	max_index = np.argmax(scores)
	min_index = np.argmin(scores)

	return max_index, min_index

print getPathIndex()








