#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, Point
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Float32MultiArray
from driving_controller import DrivingController

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic", "/pf/pose/odom")
        self.lookahead        = 1.2 # 0.7 for v=1, 1 for v=2
        # self.wheelbase_length = # FILL IN #
        
        self.pose = None
        self.target_point = None

        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.received_traj = False
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.pose_callback, queue_size=1)
        self.error_pub = rospy.Publisher("/traj_error", Float32MultiArray, queue_size=1)

        self.min_point_pub = rospy.Publisher("/min_point", PointStamped, queue_size=1)
        self.intersection_pub = rospy.Publisher("/intersection", PointStamped, queue_size=1)

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print("Receiving new trajectory:", len(msg.poses), "points")
        self.received_traj = True
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

    def pose_callback(self, odom):
        self.pose = odom.pose.pose
        robot_position = np.array([self.pose.position.x, self.pose.position.y])

        if self.received_traj:
            min_segment_index, min_point, min_dist = self.min_dist(robot_position, self.trajectory.points)
            self.min_point_pub.publish(PointStamped(point=Point(x=min_point[0], y=min_point[1], z=0), 
                                                header=Header(frame_id="map")))
            intersection_exists, intersection = self.find_intersection(min_segment_index, robot_position)
            if intersection_exists:
                self.intersection_pub.publish(PointStamped(point=Point(x=intersection[0], y=intersection[1], z=0), 
                                            header=Header(frame_id="map")))
                
            self.error_pub.publish(Float32MultiArray(data=[min_dist]))


    def min_dist(self, robot_position, trajectory_points):
        epsilon = .01
        startpoints = np.array(trajectory_points)
        endpoints = np.roll(startpoints, -1, axis=0)
        startpoints = startpoints[:-1]
        endpoints = endpoints[:-1]

        l2 = np.linalg.norm(startpoints-endpoints, axis=1)**2 # vectorized distance between each pair of points
        l2[l2 == 0] = epsilon # eliminate possibility of divide by 0 if start=end for a segment
        t = np.clip(np.sum((robot_position-startpoints)*(endpoints-startpoints), axis=1)/l2, 0, 1)
        projection = startpoints + (t * (endpoints - startpoints).T).T
        # print('p', projection)
        distance = np.linalg.norm(robot_position - projection, axis = 1)
        index = np.argmin(distance)
        min_distance = distance[index]
        min_point = projection[index]
        return (index, min_point, min_distance)
    
    def find_intersection(self, closest_index, robot_position):
        """
        Returns a point that is "lookahead distance" away from the car
        but also on the trajectory path. The search starts from the nearest
        linear trajectory segment and proceeds in order if not found
        """
        current_segment_index = closest_index
        point_exists = False
        while not point_exists: # Loop through the segments in the traj
            start_point = np.array(self.trajectory.points[current_segment_index])
            end_point = np.array(self.trajectory.points[current_segment_index+1])

            point_exists, point = self.circle_intersection(start_point, end_point, robot_position)
            if point_exists:
                return point_exists, point

            current_segment_index += 1
            if current_segment_index >= len(self.trajectory.points)-1: # We passed the last segment of the traj
                return False, None
        
    def circle_intersection(self, start_point, end_point, robot_position):
        """
        Finds the points of intersection between a circle of radius "lookahead 
        distance" and a line segment given by the start_point and end_point.
        """
        # Center of circle is robot_position: Q
        # Radius of circle is self.lookahead: r
        # Start of line segment is start_point: P1
        V = end_point - start_point  # Vector along line segment

        # Quadratic equation coefficients
        a = V.dot(V)
        b = 2 * V.dot(start_point - robot_position)
        c = (start_point.dot(start_point) + robot_position.dot(robot_position) 
             - 2 * start_point.dot(robot_position) - self.lookahead**2)
        
        disc = b**2 - 4 * a * c
        if disc < 0: # Quadratic has no real solutions
            return False, None
        
        sqrt_disc = np.sqrt(disc)
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / (2 * a)

        # TODO: Only pick points that are in front of us
        p1 = start_point + t1 * V
        p2 = start_point + t2 * V

        # TODO: If both points are in front, pick the closer one

        if 0 <= t1 <= 1: # Prioritize point that is closer to the endpoint than startpoint
            return True, p1
        else: # Move to next line segment if the one closer to endpoint is not on the segment
            return False, None

    
    def old_min_dist(self, robot_position, v, w):
        # Return minimum distance between line segment vw and point p
        # v, w are Point
        robot_position = np.array(robot_position)
        w = np.array(w)
        v = np.array(v)

        l2 = np.linalg.norm(w-v)**2  # i.e. |w-v|^2 
        if (l2 == 0.0):
            return (v, np.linalg.norm(robot_position-v))   # v == w case

        # Consider the line extending the segment, parameterized as v + t (w - v).
        # We find projection of point p onto the line. 
        # It falls where t = [(p-v) . (w-v)] / |w-v|^2
        # We clamp t from [0,1] to handle points outside the segment vw.
        t = max(0, min(1, np.dot(robot_position - v, w - v) / l2))
        projection = v + t * (w - v) # Projection falls on the segment
        return (projection, np.linalg.norm(robot_position - projection))

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    dc = DrivingController()
    rospy.spin()
