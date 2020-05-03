#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

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
        self.lookahead        = 20# FILL IN #
        self.speed            = 0.5# FILL IN #
        # self.wrap             = 0# FILL IN #
        self.wheelbase_length = 1.5# Guess #
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        # self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)

        self.drive_pub = rospy.Publisher("/tesse/drive", AckermannDriveStamped, queue_size=1)
        self.drive_msg = AckermannDriveStamped()

        rospy.Subscriber(self.odom_topic, Odometry, self.odomCallback)
        rospy.Subscriber("/tesse/hood_lidar/scan", LaserScan, self.scanCallback)
        # self.segment_num = 1

        self.arr_flag = 0
        self.odom_flag = 0
        self.scan_flag = 0

    def get_index(self, angle1, angle2, angle_min, angle_increment):
        k1 = int ((angle1 - angle_min )/ angle_increment)
        k2 = int ((angle2 - angle_min )/ angle_increment)

        return k1, k2, np.linspace(angle1, angle2, k2-k1, endpoint=True)

    def find_nearest(self, value):
        array = self.angles
        idx = (np.abs(array - value)).argmin()
        return idx

    def get_curvature(self, curvature, msg):
        for i in range(self.find_nearest(curvature), len(angles)):
            ix1, ix2, front_angs = self.get_index(self.angles_minus[i], self.angles_plus[i], msg.angle_min, msg.angle_increment)
            A = np.array([front_angs, [1]*len(front_angs)]).T
            x_hat = np.dot(np.linalg.inv(np.dot(A.T, A)), np.dot(A.T, msg.ranges[ix1:ix2]))
            lsq_ranges = np.dot(A, x_hat)
            low_mean_min = np.mean(sorted(lsq_ranges)[:(ix2-ix1)//2])
            del A, x_hat, lsq_ranges, ix1, ix2
            if low_mean_min > self.lookahead:
                print(self.angles[i])
                break
            else:
                ix1, ix2, front_angs = self.get_index(-self.angles_plus[i], -self.angles_minus[i], msg.angle_min, msg.angle_increment)
                A = np.array([front_angs, [1]*len(front_angs)]).T
                x_hat = np.dot(np.linalg.inv(np.dot(A.T, A)), np.dot(A.T, msg.ranges[ix1:ix2]))
                lsq_ranges = np.dot(A, x_hat)
                low_mean_min = np.mean(sorted(lsq_ranges)[:(ix2-ix1)//2])
                del A, x_hat, lsq_ranges, ix1, ix2
                if low_mean_min > lookahead:
                    print(self.angles[i])
                    break
        x_body = self.lookahead*np.sin(self.angles[i])
        y_body = self.lookahead*np.cos(self.angles[i])
        R_track = 2*x_body/ self.lookahead**2
        curvature = np.arctan(self.wheelbase_length/R_track)
        del R_track, x_body, y_body
        return curvature

    def scanCallback(self, msg):
        ranges = np.array(msg.ranges)
        if not self.scan_flag:
            angle = np.arctan(1/self.lookahead)
            self.angles = np.linspace(0, msg.angle_max - angle, 50)
            self.angles_plus = self.angles + angle
            self.angles_minus = self.angles - angle
            del angle
            self.scan_flag = 1

        curvature = None

        if self.arr_flag and self.odom_flag:
            ix_min = self.find_closest_point(self.arr, self.loc) #Index of closest line segment on trajectory to the car
            try:
        		#rospy.loginfo('Start math')
        		#rospy.loginfo(ix_min)
                curvature = self.find_lookahead(self.arr, ix_min, self.loc, self.quat, self.lookahead) #curvature (steering angle) in global frame
                # self.drive_msg.drive.speed = self.speed
                # self.drive_msg.drive.steering_angle = curvature
                # self.drive_pub.publish(self.drive_msg)
                #rospy.loginfo('Published drive')

            except UnboundLocalError:
                #
                # curvature = None
                pass

        if curvature:
            curvature = self.get_curvature(curvature, msg)
            self.drive_msg.drive.speed = self.speed
            self.drive_msg.drive.steering_angle = curvature
            self.drive_pub.publish(self.drive_msg)
        else:
            # self.lookahead = self.lookahead**1.2
            self.drive_msg.drive.speed = 5
            self.drive_msg.drive.steering_angle = 0
            self.drive_pub.publish(self.drive_msg)


    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
	#rospy.loginfo('Start Traj callback')

        #print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        self.segment_num = min(len(msg.poses), 5000)

        #Convert trajectory PoseArray to arr (line segment array)
        N = self.segment_num // (len(msg.poses) - 1)
        rospy.loginfo(N)
        self.arr = self.create_segment(msg.poses, N)
        self.arr_flag = 1
        # rospy.loginfo(arr)

    def odomCallback(self, msg):
	#rospy.loginfo('Start Odom callback')

        self.loc = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.quat = msg.pose.pose.orientation
        if not self.odom_flag:
            self.odom_flag = 1

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


    def find_lookahead(self, arr, ix_min, loc, quat, L_dist, debug=False):
	rospy.loginfo('ix_min')
	rospy.loginfo(ix_min)


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


	    delta_theta_1 = None
	    delta_theta_2 = None

            if ((t1 < 1) & (t1 > 0)): #1st possible global frame intersection location
                lookahead_global_1 = P1 + t1*V

		lookahead_local_1 = lookahead_global_1 - loc
   		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])
		lookahead_theta = np.arctan2(lookahead_local_1[1], lookahead_local_1[0])
		delta_theta_1 = lookahead_theta - yaw
		if delta_theta_1 < -np.pi:
			delta_theta_1 += 2*np.pi
		if delta_theta_1 > np.pi:
			delta_theta_1 -= 2*np.pi

                #break
            if ((t2 < 1) & (t2 > 0)): #2nd possible global frame intersection location
                lookahead_global_2 = P1 + t2*V
                #break

		lookahead_local_2 = lookahead_global_2 - loc
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])
		lookahead_theta = np.arctan2(lookahead_local_2[1], lookahead_local_2[0])
		delta_theta_2 = lookahead_theta - yaw

		if delta_theta_2 < -np.pi:
			delta_theta_2 += 2*np.pi
		if delta_theta_2 > np.pi:
			delta_theta_2 -= 2*np.pi

	    #Expecting delta theta values to be -3.1 to 3.1, with 0 being right on
	    #rospy.loginfo('delta_theta_1')
	    #rospy.loginfo(delta_theta_1)

	    #rospy.loginfo('delta_theta_2')
	    #rospy.loginfo(delta_theta_2)

	    #If both exist, must be ix_min, so choose the theta more in car's direction
	    if delta_theta_1 and delta_theta_2:
		if abs(delta_theta_1) < abs(delta_theta_2):
			delta_theta = delta_theta_1
			break
		else:
			delta_theta = delta_theta_2
			break

	    #If one exists, if it's within a 90 deg swath either way, use it.  Otherwise next segment.
	    elif delta_theta_1:
		if abs(delta_theta_1) < np.pi/2:
			delta_theta = delta_theta_1
			break
		else:
			continue

	    #If one exists
	    elif delta_theta_2:
		if abs(delta_theta_2) < np.pi/2:
			delta_theta = delta_theta_2
			break
		else:
			continue

	    #If no intersection exists, go to next segment
	    else:
		continue


		#self.robotMarker.pose.position.x = lookahead_global_1[0]
		#self.robotMarker.pose.position.y = lookahead_global_1[1]
		#self.markerPub.publish(self.robotMarker)

		#rospy.loginfo('aim point global 1:')
		#rospy.loginfo(lookahead_global_1)



	#rospy.loginfo('delta_theta')
	#rospy.loginfo(delta_theta)


	rad_curvature = 0.5 * L_dist / np.sin(delta_theta) #Rad. of curvature needed to hit lookahead point
	curvature = np.arctan(self.wheelbase_length / rad_curvature)

	#rospy.loginfo('curvature')
	#rospy.loginfo(curvature)


        if debug:
            return lookahead_global, curvature
        return curvature, delta_theta #- np.pi


if __name__=="__main__":
    rospy.init_node("pursuit_avoid")
    pf = PursuitAvoid()
    rospy.spin()
