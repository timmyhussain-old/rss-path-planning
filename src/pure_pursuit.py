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
        self.speed            = 0.5# FILL IN #
        self.wrap             = 0# FILL IN #
        self.wheelbase_length = 0# FILL IN #
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.drive_msg = AckermannDriveStamped()
        rospy.Subscriber(self.odom_topic, Odometry, self.odomCallback)
        self.segment_num = 100
        self.arr_flag = 0


    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        #print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        #Convert trajectory PoseArray to arr (line segment array)
        N = self.segment_num // (len(msg.poses) - 1)
        rospy.loginfo(N)
        self.arr = self.create_segment(msg.poses, N)
        self.arr_flag = 1
        # rospy.loginfo(arr)

    def odomCallback(self, msg):
        loc = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

        if self.arr_flag:
            ix_min = self.find_closest_point(self.arr, loc) #Index of closest line segment on trajectory to the car
            try:
                curvature = self.find_lookahead(self.arr, ix_min, loc, self.lookahead) #curvature (steering angle) in global frame
                self.drive_msg.drive.speed = self.speed
                self.drive_msg.drive.steering_angle = curvature
                self.drive_pub.publish(self.drive_msg)
            except UnboundLocalError:
                # self.lookahead = self.lookahead + 1.0
                pass

        #Publish curvature/steering angle and self.speed to "/drive"

    def create_segment(self, arr, N):
        '''
        inputs:
        -numpy array of
        '''
    #     a = arr[0]
    #     b = arr[1]
    #     np.linspace(a.position.x, b.position.x, N)
    #     np.linspace(a.position.y, b.position.y, N)
        x = []
        y = []
        for i in range(len(arr) - 2):
            a = arr[i]
            b = arr[i+1]
            x = x + np.linspace(a.position.x, b.position.x, N, endpoint=False).tolist()
            y = y + np.linspace(a.position.y, b.position.y, N, endpoint=False).tolist()
        a = arr[i+1]
        b = arr[i+2]
        x = x + np.linspace(a.position.x, b.position.x, N, endpoint=True).tolist()
        y = y + np.linspace(a.position.y, b.position.y, N, endpoint=True).tolist()

        segments = []
        for i in range(len(x) - 1):
            segments.append([[x[i], y[i]], [x[i+1], y[i+1]]])
        return np.array(segments)

    def find_closest_point(self, arr, loc):
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


    def find_lookahead(self, arr, ix_min, loc, L_dist, debug=False):
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
        curvature = lookahead_local[0] * 2.0/L_dist**2
        if debug:
            return lookahead_global, curvature
        return curvature #- np.pi


if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
