#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import time
import utils
import tf
import os

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

class PursuitAvoid(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = "/pf/pose/odom"
        self.lookahead        = 10# FILL IN #
        self.speed            = 30 # FILL IN #
        # self.wrap             = 0# FILL IN #
        self.wheelbase_length = 2.5# Guess #
	
	# FOR TESTING ----------------------------------------------
	#self.save_path = os.path.join(lab6_path+"/trajectories/", time.strftime("%Y-%m-%d-%H-%M-%S") + "-test.txt")
	#-----------------------------------------------------------
		
        # self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        # self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)

        #**********************************************************************************************************
        #load trajectory
        self.path = rospy.get_param("~race_trj")
        rospack = rospkg.RosPack()
        self.lab6_path = rospack.get_path("lab6")
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        # self.trajectory.publish_trajectory()
        self.trajectory.load(self.path)
        self.trajectory.load(os.path.join(self.lab6_path+"/trajectories/race_path.traj"))
        self.segment_num = max(len(self.trajectory.points), 1000)

        #Convert trajectory PoseArray to arr (line segment array)
        N = self.segment_num // (len(self.trajectory.points) - 1)
        #rospy.loginfo(N)
        self.arr = self.create_segment(self.trajectory.toPoseArray().poses, N)
        #rospy.loginfo(self.arr)
        #***********************************************************************************************************

        self.drive_pub = rospy.Publisher("/tesse/drive", AckermannDriveStamped, queue_size=1)
        self.drive_msg = AckermannDriveStamped()

        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odomCallback)
        self.last_ix = 0
	self.last_lookahead = 0

	self.thetas = []

        self.steering_angle_arr = []
        self.previous_steering = 0
        self.time_now = time.time()
        rospy.on_shutdown(self.save_arrays)


    def save_arrays(self):
        np.savetxt(os.path.join(self.lab6_path +"/src/steering.txt"), np.array(self.steering_angle_arr))
        # np.savetxt(os.path.join(self.lab6_path +"/src/steering_dt.txt"), np.array(self.steering_angle_dt))

    def odomCallback(self, msg):
        # rospy.loginfo('Start Odom callback')

        self.loc = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.quat = msg.pose.pose.orientation

        curvature = None
        ix_min = self.find_closest_point(self.arr, self.loc) #Index of closest line segment on trajectory to the car
        #rospy.loginfo(ix_min)
        self.last_ix = ix_min
        try:
	    self.thetas = self.find_lookahead_thetas(self.arr, ix_min, self.loc, self.quat, [10, 55])
	
	    theta_pow = np.power(self.thetas, 2.5)

	    self.speed = max(8, 46 - np.mean(theta_pow)*130)

	    lookahead = max(5, self.speed * 1.7)
	    lookahead = min(lookahead, 24)
	    lookahead = (lookahead + 3*self.last_lookahead)/4 #speed doesn't change instantly, so lookahead shouldn't

	    self.last_lookahead = lookahead

	    #rospy.loginfo("lookahead: %f", lookahead)
	    #rospy.loginfo("speed: %f", self.speed)

            curvature = self.find_lookahead(self.arr, ix_min, self.loc, self.quat, lookahead) #curvature (steering angle) in global frame

        except UnboundLocalError:
            #
            # curvature = None
            #print("too far from lookahead point")
            pass

        if curvature is not None:
            delt = 0.2
            if time.time()- self.time_now > delt:
                dt = time.clock()-self.time_now
                # if (-53 < self.loc[0] < 1.65) and (80 < self.loc[1] < 153):
                #     self.lookahead = 40
                #     self.drive_msg.drive.speed = 10
                #     self.drive_msg.drive.steering_angle = 0.2*curvature +  0.08*(curvature - self.previous_steering)/delt
                # else:
		
                self.drive_msg.drive.speed = self.speed
		
                self.drive_msg.drive.steering_angle = 0.08*curvature +  0.002*(curvature - self.previous_steering)/delt
		
		print(str(self.speed) + ";" + str(self.loc))
                self.drive_pub.publish(self.drive_msg)
                self.steering_angle_arr.append(self.drive_msg.drive.steering_angle)
                # self.previous_steering =
                self.previous_steering = self.drive_msg.drive.steering_angle
                self.time_now = time.time()
        else:
            # self.lookahead = self.lookahead**1.2
            self.drive_msg.drive.speed = 8.0
            self.drive_msg.drive.steering_angle = 0
	    print(str(8.0) + ";" + str(self.loc))
            self.drive_pub.publish(self.drive_msg)
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

        # dist_ix = np.arange(len(dist_array))
        # dist_ix = np.where(dist_ix - self.last_ix < 0, 10000, dist_ix)
        # return np.argmin(dist_ix * dist_array)


        ix_flag = 1
        while ix_flag:
            ix_min = np.argmin(dist_array) #get the index of the line segment the point is closest to
            dist_array = np.delete(dist_array, ix_min)
            if abs(ix_min - self.last_ix) < 100:
                ix_flag = 0

        #Might not need this
        #closest_point = np.array([x_array[min_dist_idx], y_array[min_dist_idx]]) #get the point car is closest to

        return ix_min


    def find_lookahead(self, arr, ix_min, loc, quat, L_dist, debug=False):
	# rospy.loginfo('ix_min')
	# rospy.loginfo(ix_min)


        for i in range(ix_min, arr.shape[0], 1):
            Q = loc                # Centre of circle is vehicle location
            r = L_dist             # Radius of circle is lookahead distance
            P1 = arr[i, 0, :]      # Start of line segment
            V = arr[i, 1, :] - P1  # Line segment vector (the endpoint if starts at origin)

            a = V.dot(V)
            b = 2 * V.dot(P1 - Q)
            c = P1.dot(P1) + Q.dot(Q) - 2 * P1.dot(Q) - r**2


            disc = b**2 - 4 * a * c
            if disc < 0:
                continue #circle doesn't intersect this line segment, try next one

	    #Fractions along the line segment where circle intersects
            t1 = (-b + np.sqrt(disc)) / (2.0 * a)
            t2 = (-b - np.sqrt(disc)) / (2.0 * a)

            # rospy.loginfo("t1: " + str(t1) + " t2: " + str(t2) + "\n" )
            if ((t1 < 1) & (t1 > 0)): #1st possible global frame intersection location

                lookahead_global = P1 + t1*V
                break

            elif ((t2 < 1) & (t2 > 0)): #2nd possible global frame intersection location
                lookahead_global = P1 + t2*V
                break

        lookahead_local = lookahead_global - loc

        roll, pitch, yaw = tf.transformations.euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])
        yaw = yaw - np.pi/2
        R = np.array([[np.cos(yaw), np.sin(yaw)], \
                [-np.sin(yaw), np.cos(yaw)]])
        # rospy.loginfo(R)
        #rospy.loginfo(np.dot(R, lookahead_local))
        # self.odom_sub.unregister()
        # rospy.loginfo("here")
        r_track = (L_dist**2)/np.dot(R, lookahead_local)[0]
        # rospy.loginfo("here")
        curvature = np.arctan(self.wheelbase_length/r_track)
        # rospy.loginfo(curvature)
        return curvature



    def find_lookahead_thetas(self, arr, ix_min, loc, quat, L_dists, debug=False):
	# rospy.loginfo('ix_min')
	# rospy.loginfo(ix_min)

	thetas = []

	for L_dist in L_dists:

		for i in range(ix_min, arr.shape[0], 1):
		    Q = loc                # Centre of circle is vehicle location
		    r = L_dist             # Radius of circle is lookahead distance
		    P1 = arr[i, 0, :]      # Start of line segment
		    V = arr[i, 1, :] - P1  # Line segment vector (the endpoint if starts at origin)

		    a = V.dot(V)
		    b = 2 * V.dot(P1 - Q)
		    c = P1.dot(P1) + Q.dot(Q) - 2 * P1.dot(Q) - r**2

		    disc = b**2 - 4 * a * c
		    if disc < 0:
		        continue #circle doesn't intersect this line segment, try next one

		    #Fractions along the line segment where circle intersects
		    t1 = (-b + np.sqrt(disc)) / (2.0 * a)
		    t2 = (-b - np.sqrt(disc)) / (2.0 * a)

		    # rospy.loginfo("t1: " + str(t1) + " t2: " + str(t2) + "\n" )
		    if ((t1 < 1) & (t1 > 0)): #1st possible global frame intersection location

		        lookahead_global = P1 + t1*V
		        break

		    elif ((t2 < 1) & (t2 > 0)): #2nd possible global frame intersection location
		        lookahead_global = P1 + t2*V
		        break

		lookahead_local = lookahead_global - loc

		roll, pitch, yaw = tf.transformations.euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])
		yaw = yaw - np.pi/2
		R = np.array([[np.cos(yaw), np.sin(yaw)], \
		        [-np.sin(yaw), np.cos(yaw)]])
	 
		rel_lookahead_point = np.dot(R, lookahead_local)
		theta = abs(np.arctan2(rel_lookahead_point[0], rel_lookahead_point[1]))
		thetas.append(theta)
	
	#rospy.loginfo(thetas)

        return thetas


if __name__=="__main__":
    rospy.init_node("pursuit_avoid")
    pf = PursuitAvoid()
    rospy.spin()
