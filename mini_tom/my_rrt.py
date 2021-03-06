import numpy as np
import time

STEP_SIZE = .01

# python -m cProfile -s tottime my_rrt.py

def pool_test( x ):
    return np.random.rand()

# adapted from the original matlab code at
# http://www.mathworks.com/matlabcentral/fileexchange/27205-fast-line-segment-intersection
def line_intersect( X1,Y1,X2,Y2, X3,Y3,X4,Y4 ):
    X4_X3 = X4.T - X3.T
    Y1_Y3 = Y1   - Y3.T
    Y4_Y3 = Y4.T - Y3.T
    X1_X3 = X1   - X3.T
    X2_X1 = X2   - X1
    Y2_Y1 = Y2   - Y1

    numerator_a = X4_X3 * Y1_Y3 - Y4_Y3 * X1_X3
    numerator_b = X2_X1 * Y1_Y3 - Y2_Y1 * X1_X3
    denominator = Y4_Y3 * X2_X1 - X4_X3 * Y2_Y1

    u_a = numerator_a / (denominator+1e-20)
    u_b = numerator_b / (denominator+1e-20)

    INT_X = X1 + X2_X1 * u_a
    INT_Y = Y1 + Y2_Y1 * u_a
    did_intersect = (u_a >= 0) & (u_a <= 1) & (u_b >= 0) & (u_b <= 1)

    return INT_X, INT_Y, did_intersect


#
# loads a set of polygons
#
# note that this appends the first point to also be the last point.
# this function assumes that the list is given in "open" form; by
# appending the first point as the last point, it ensures that the
# resulting polygon is exactly closed.
#
# note that this prepends a single point that is the mean of all the other points.
# this is for drawing the polygons using a GL_TRIANGLE_FAN.  It's a total hack.
#
def load_polygons( fn="./paths.txt" ):
    bdata = []
    for x in open( fn ):
        tmp = np.fromstring( x, dtype=float, sep=' ' )
        tmp = np.reshape( tmp/1000.0, (-1,2) )
        tmp = np.vstack(( np.mean(tmp, axis=0, keepdims=True), tmp, tmp[0,:] ))
        tmp[:,1] = 1.0 - tmp[:,1]  # flip on the y axis
        bdata.append( tmp )
    return bdata

# polygon_list is a list of np arrays
# each nparray is a kx2 matrix, representing x,y points
# first entry is the mean of all the points, which is SKIPPED
# last entry in the matrix is the same as the first
# returns x1,y1, x2,y2
def polygons_to_segments( polygon_list ):
    X1 = []
    Y1 = []
    X2 = []
    Y2 = []
    for x in polygon_list:
        X1.append( x[1:-1,0:1] )
        Y1.append( x[1:-1,1:2] )
        X2.append( x[2:,0:1] )
        Y2.append( x[2:,1:2] )
    X1 = np.vstack( X1 )
    Y1 = np.vstack( Y1 )
    X2 = np.vstack( X2 )
    Y2 = np.vstack( Y2 )

    return X1, Y1, X2, Y2

def distance_to_other_points( pt, pts ):
    diffs = (pts - pt)**2.0
    return np.sum( diffs, axis=1, keepdims=True )

def run_rrt_poly( start_pt, goal_pt, polygons, bias=0.75, plot=False, step_limit=20000, scale=1):
    '''
    start_pt: 1 x 2 np array
    goal_pt: 1 x 2 np array
    polygons: list (polygons) of n x 2 (x, y) np arrays
    bias: 

    returns a list of length 2 np arrays describing the path from `start_pt` to `goal_pt`
    '''
    x1, y1, x2, y2 = polygons_to_segments( polygons )
    return run_rrt( start_pt, goal_pt, x1, y1, x2, y2, bias, plot, scale=scale )

def run_rrt( start_pt, goal_pt, endpoint_a_x, endpoint_a_y, endpoint_b_x, endpoint_b_y,  bias=0.75, plot=False, step_limit=20000, scale=1 ):
    nodes = start_pt
    parents = np.atleast_2d( [0] )

    for i in range( 0, step_limit ):
        random_point = np.random.rand(1,2) * scale

        # find nearest node
        distances = distance_to_other_points( random_point, nodes )
        nearest_ind = np.argmin( distances )
        nearest_point = nodes[ nearest_ind:nearest_ind+1, : ]

        # take a step towards the goal
        if np.random.rand() > bias:
            ndiff = goal_pt - nearest_point
        else:
            ndiff = random_point - nearest_point

        ndiff = (scale * STEP_SIZE) * ndiff / np.sqrt( np.sum( ndiff*ndiff ) + 1e-20 )
        new_pt = nearest_point + ndiff

        if distance_to_other_points( new_pt, goal_pt ) < (.005 * scale):
            #print('i', i)
            path = [ new_pt[0,:] ]
            while nearest_ind != 0:
                path.append( nodes[nearest_ind,:] )
                nearest_ind = parents[ nearest_ind, 0 ]
            path.append( nodes[0,:] )

            if plot == True:
                plt.figure()
                for i in range(0, endpoint_a_x.shape[0]):
                    plt.plot( [ endpoint_a_x[i], endpoint_b_x[i] ], [ endpoint_a_y[i], endpoint_b_y[i] ], 'k' )
                for i in range( 0, len(path)-1 ):
                    plt.plot( [ path[i][0], path[i+1][0] ], [ path[i][1], path[i+1][1] ], 'b' )
                plt.scatter( start_pt[0,0], start_pt[0,1] )
                plt.scatter( goal_pt[0,0], goal_pt[0,1] )
                plt.show()

            path.reverse()

            return path

        # we'd like to expand from nearest_point to new_pt.  Does it cross a wall?
        int_x, int_y, intersection_indicators = line_intersect( 
            nearest_point[0,0], 
            nearest_point[0,1], 
            new_pt[0,0], 
            new_pt[0,1], 
            endpoint_a_x, endpoint_a_y, endpoint_b_x, endpoint_b_y)

        if intersection_indicators.any():
            # calculate nearest intersection and trim new_pt
            intersections = np.atleast_2d( [ int_x[intersection_indicators], int_y[intersection_indicators] ] ).T
            distances = distance_to_other_points( nearest_point, intersections )
            closest_intersection_index = np.argmin( distances )
            new_pt = intersections[ closest_intersection_index:closest_intersection_index+1, : ]
            safety = new_pt - nearest_point
            safety = scale * 0.001 * safety / np.sqrt( np.sum( safety*safety ) + 1e-20 )
            new_pt = new_pt - safety

        nodes = np.vstack(( nodes, new_pt ))
        parents = np.vstack(( parents, nearest_ind ))

    #print('No path found!')
    return []

# ==============================================================

if __name__ == '__main__':
    polygons = load_polygons( "./paths.txt" )
    
    start_pt = np.atleast_2d( [0.1,0.1] )
    goal_pt = np.atleast_2d( [0.9,0.9] )

    path = run_rrt_poly( start_pt, goal_pt, polygons, plot=True)

mypath = [0.88326248,  0.88557536,
0.90324988,  0.8397441 ,
0.90592584,  0.78981576,
0.91337393,  0.74037361,
0.89314996,  0.69464625,
0.87720748,  0.64725599,
0.84693485,  0.60746192,
0.8380107 ,  0.55826477,
0.82534527,  0.60663405,
0.77606357,  0.59818928,
0.77522948,  0.59746264,
0.76008271,  0.5607355 ,
0.75786194,  0.55931555,
0.73863468,  0.56273596,
0.71705482,  0.51763264,
0.70284157,  0.54058654,
0.65454237,  0.52765644,
0.62702284,  0.48591108,
0.60543529,  0.53101072,
0.56923513,  0.56550055,
0.57538613,  0.61512075,
0.53780565,  0.58214034,
0.50022517,  0.54915993,
0.45249981,  0.5640693 ,
0.45265718,  0.56497348,
0.45020553,  0.56484287,
0.44971871,  0.56471506,
0.44916269,  0.56447462,
0.44914448,  0.56445873,
0.42321731,  0.54076924,
0.44489987,  0.49571519,
0.44481103,  0.49528903,
0.44471408,  0.49496751,
0.4445358 ,  0.49453663,
0.44441105,  0.49434999,
0.43271649,  0.48166347,
0.43169384,  0.43167393,
0.43187455,  0.43156532,
0.4261321 ,  0.42226862,
0.37620231,  0.42491743,
0.37434465,  0.42323254,
0.36976347,  0.38689823,
0.32021466,  0.39360015,
0.32009459,  0.3933292 ,
0.31767863,  0.3788699 ,
0.2682576 ,  0.37128298,
0.26932061,  0.32129428,
0.28193985,  0.27291293,
0.28190038,  0.27286092,
0.2746515 ,  0.26464885,
0.29248369,  0.21793683,
0.29353604,  0.2070827 ,
0.29405522,  0.20703691,
0.29399146,  0.20683259,
0.29383236,  0.20647408,
0.29364819,  0.20626337,
0.28316176,  0.204748  ,
0.23397609,  0.19576078,
0.22907469,  0.19426936,
0.20183268,  0.16561413,
0.15376008,  0.17936278,
0.14781675,  0.17766148,
0.14738945,  0.17723302,
0.13535534,  0.13535534,
0.1,  0.1]
# ECNT = 100
# biases = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9]
# results = np.zeros(( len(biases), ECNT ))
# for bi,b in enumerate( biases ):
#     for k in range(0,ECNT):
#         nodes, parents = run_rrt( start_pt, goal_pt, endpoint_a_x,endpoint_a_y,endpoint_b_x,endpoint_b_y, bias=b )
#         results[bi,k] = nodes.shape[0]
#     print "%.2f: mean=%.2f (var=%.2f)" % ( biases[bi], np.mean( results[bi,:] ), np.var( results[bi,:]) )
# plt.figure()
# for i in range(0,endpoint_a_x.shape[0]):
#     plt.plot( [ endpoint_a_x[i], endpoint_b_x[i] ], [ endpoint_a_y[i], endpoint_b_y[i] ], 'k' )
# for i in range(0,nodes.shape[0]):
#     plt.plot( [nodes[i,0],nodes[parents[i],0]], [nodes[i,1],nodes[parents[i],1]], 'b' )
# plt.scatter( start_pt[0,0], start_pt[0,1] )
# plt.scatter( goal_pt[0,0], goal_pt[0,1] )
# plt.show()
