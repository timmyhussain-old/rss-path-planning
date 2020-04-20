#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic")
        self.lookahead        = 1# FILL IN #
        self.speed            = 0# FILL IN #
        self.wrap             = 0# FILL IN #
        self.wheelbase_length = 0# FILL IN #
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        #print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        #Convert trajectory PoseArray to arr (line segment array)
        #arr = 

        #Get location of robot in global frame (call particle filter?)
        #loc = 

        ix_min = find_closest_point(arr, loc) #Index of closest line segment on trajectory to the car

        curvature = find_lookahead(arr, ix_min, loc, self.lookahead) #curvature (steering angle) in global frame

        #Publish curvature/steering angle and self.speed to "/drive"


    def find_closest_point(arr, loc):
    """
    inputs:
    -arr = N x 2 x 2 numpy array with each segment of the trajectory of form [[x1, y1],[x2, y2]]
    -loc = [x, y] numpy array representing car location in global frame

    Might not need this -> output: closest_point = [x, y] numpy array representing closest point on trajectory to car
    output: index of closest line segment
    """

        line_dx_array = arr[:,1,0] - arr[:,0,0] #line segment x deltas
        line_dy_array = arr[:,1,1] - arr[:,0,1] #line segment y deltas

        norms = np.square(line_dx_array) + np.square(line_dy_array)


        point_dx1 = loc[0] - arr[:,0,0] #x dist from point to start of segment
        point_dy1 = loc[1] - arr[:,0,1] #y dist from point to start of segment

        line_frac_array = (point_dx1 * line_dx_array + point_dy1 * line_dy_array) / norms #fractions along the line
        line_frac_array = np.clip(line_frac_array, a_min = 0, a_max = 1) #clipped so stays "on" the line segment

        x_array = arr[:,0,0] + line_frac_array * line_dx_array #x locations of closest points on line segments
        y_array = arr[:,0,1] + line_frac_array * line_dy_array #y locations of closest points on line segments


        dx_array = x_array - loc[0] #x dists to closest point on segment
        dy_array = y_array - loc[1] #y dists to closest point on segment

        dx2_array = np.square(dx_array) #squared x dists
        dy2_array = np.square(dy_array) #squared y dists

        dist_array = np.add(dx2_array, dy2_array) #array of dists from car location to closest point on all segments

        ix_min = np.argmin(dist_array) #get the index of the line segment the point is closest to

        #Might not need this
        #closest_point = np.array([x_array[min_dist_idx], y_array[min_dist_idx]]) #get the point car is closest to

        return ix_min


    def find_lookahead(arr, ix_min, loc, L_dist, debug=False):
    '''
    inputs:
    -arr = N x 2 x 2 numpy array where each segment is of the form [[x1, y1],
                                                                    [x2, y2]]
    -ix_min = index of closest segment
    -loc = location of the car in global frame
    -L_dist = lookahead distance
     
    output: curvature (steering angle) in global frame
    '''
        for i in range(ix_min, arr.shape[0], 1):
            Q = loc                 # Centre of circle
            r = L_dist                 # Radius of circle
            P1 = arr[i, 0, :]      # Start of line segment
            V = arr[i, 1, :] - P1 
            a = V.dot(V)
            b = 2 * V.dot(P1 - Q)
            c = P1.dot(P1) + Q.dot(Q) - 2 * P1.dot(Q) - r**2
            
            disc = b**2 - 4 * a * c
            if disc < 0:
                continue
            t1 = (-b + np.sqrt(disc)) / (2.0 * a)
            t2 = (-b - np.sqrt(disc)) / (2.0 * a)
            if ((t1 < 1) & (t1 > 0)):
                lookahead_global = P1 + t1*V
    #             print(i)
                break
            elif ((t2 < 1) & (t2 > 0)):
                lookahead_global = P1 + t2*V
                break
        lookahead_local = lookahead_global - loc
        curvature = lookahead_local[0] * 2.0/L_dist
        if debug:
            return lookahead_global, curvature
        return curvature


if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()