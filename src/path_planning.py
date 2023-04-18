#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
import tf.transformations as tf
from utils import LineTrajectory

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.static_map = True
        self.map = None
        self.map_height = 0
        self.map_width = 0
        self.map_resolution = 
        self.goal_pose = None
        self.current_pose = None

        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)


        self.trajectory = LineTrajectory("/planned_trajectory")
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)

        # use 2 dictionaries to manage rrt graph structure / path reconstruction
        self.tree = {} # parent : set(children) --- actually maybe don't need this?
        self.parents = {} # child : parent


    def map_cb(self, map_msg): 
        #should we use rospy.wait_for_message instead? (do we need to get map message more than once?)
            # map will be updated if there are moving obstacles

        #normalize and clip map values to [0, 1]
        # we shouldn't clip values since -1 indicates positions we can't visit (outside of the walls)
        # self.map = np.array(map_msg.data, np.double)/100.
        # self.map = np.clip(self.map, 0, 1)

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.euler_from_quaternion((
                origin_o.x,
                origin_o.y,
                origin_o.z,
                origin_o.w))
        self.map_origin = (origin_p.x, origin_p.y, origin_o[2])

        if self.map != None and self.static_map == True: #only run callback until we get the map
            return

        self.map = np.array(map_msg.data).reshape(map_msg.info.height, map_msg.info.width) # index map as grid[y direction, x direction]
        
        self.map_height = map.info.height
        self.map_width = map.info.width
        self.map_resolution = map.info.resolution # meters / cell
        # self.map_origin = 
        pass ## REMOVE AND FILL IN ##


    def odom_cb(self, msg):
        self.current_pose = msg.pose
        pass ## REMOVE AND FILL IN ##


    def goal_cb(self, msg):
        self.goal_pose = msg.pose
        pass ## REMOVE AND FILL IN ##

    def cell_to_world(self, u, v):
        angle = self.map_origin[2]
        rotation_matrix = np.array([[np.cos(angle), np.sin(angle), 0], 
                                    [-np.sin(angle), np.cos(angle), 0], 
                                    [0, 0, 1]])
        rotated_coord =  np.matmul(rotation_matrix, np.array([u * self.map_resolution + self.map_origin[0], v * self.map_resolution + self.map_origin[1], 0]))
        
        return (rotated_coord[0], rotated_coord[1])



    ### helpers for rrt alg ###
    def sample_map(self):
        """
        - map resolution is 0.05 meters / cell for Stata basement (relatively high resolution)
            - width = 1730 pixels
            - height = 1300 pixels

        so let's sample pixels instead of real, continuous coordinates
        Args:
            map (_type_): _description_
        """
        
        while True:
            rand_indices = np.random.random(0, [self.map_height, self.map_width])
            v, u = rand_indices[0], rand_indices[1] #y direction, x direction

            if self.point_collision_check(map, rand_indices) == True: # collision happens
                continue

            else:
                return self.cell_to_world(u, v)
            
        # pick next random point
        
        # TODO: convert to a point using map dimensions
        pass

    def point_collision_check(self, v, u):
        # return True if collision exists at this cell
        # grid_ind = None #grid_ind should be the ind of the 1d occupancy grid cell the position belongs in
        return self.map[v][u] != 0

    def path_collision_check(self, map, start, end):
        # check that the path between start, end is collision free - identify occupancy grid squares affected and check each
        pass

    def find_nearest_vertex(self, graph, position):
        # find nearest vertex to a given position
        # simplest heuristic: euclidean distance
        # could consider others like spline? dubins path? -- this can be an optimization task
        # iterate through node list and identify the one with lowest distance
        pass

    def reached_goal(self, node, end_point):

        #if node is near end_point, return true. Else return false

        pass

    ### rrt alg ###
    def plan_path(self, start_point, end_point, map, max_distance):
        #TODO Add max_distance parameter, new nodes should not exceed a certain distance from their nearest node
        ## CODE FOR PATH PLANNING ##
        goal_reached = False
        max_iter = 100_000
        current_iter = 0
        while not goal_reached and current_iter <= max_iter :
            node_new = self.sample_map(map) # point collision check is perfomed in sampling (only return valid samples)

            node_nearest = self.find_nearest_vertex(self.tree, node_new)
            if self.path_collision_check(map, node_new, node_nearest):
                continue

            # update tree
            self.tree[node_new] = set() # initialize new node in tree
            self.tree[node_nearest].add(node_new) # add new node to child set of node_nearest
            self.parents[node_new] = node_nearest

            # if node_new == end_point:
            #     goal_reached = True

            if self.reached_goal(node_new, end_point):
                goal_reached = True
                break

            current_iter += 1

        # by this point, the tree has been constructed
        # reconstruct trajectory given graph by recursing thru self.parents
        reverse_path = [end_point] # points along traj in reverse order

        current_node = end_point
        while current_node != start_point:
            current_node = self.parents[end_point] # backtrack to parent node
            reverse_path.append(current_node)

        for pt in reverse_path[::-1]: # populate trajectory object
            self.trajectory.addPoint(pt)

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
