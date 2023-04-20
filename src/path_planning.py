#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid, Path
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
        self.map_resolution = None
        self.current_pose = None
        self.goal_pose = None
        self.acceptable_goal_cells = None

        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)


        self.trajectory = LineTrajectory("/planned_trajectory")
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=1)

        # use 2 dictionaries to manage rrt graph structure / path reconstruction
        # self.tree = {} # parent : set(children) --- actually maybe don't need this, not really using it
        self.parents = {} # child : parent

        # plan path
        # while self.map == None or self.goal_pose == None or self.current_pose == None:
        #     rospy.spin()
            
        # self.plan_path(self.current_pose, self.goal_pose, self.map, None)


    def map_cb(self, map_msg): 
        #should we use rospy.wait_for_message instead? (do we need to get map message more than once?)
            # map will be updated if there are moving obstacles

        #normalize and clip map values to [0, 1]
        # we shouldn't clip values since -1 indicates positions we can't visit (outside of the walls)
        # self.map = np.array(map_msg.data, np.double)/100.
        # self.map = np.clip(self.map, 0, 1)

        # Convert the origin to a tuple
        print("doing map callback now")
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
        # print(self.map)

        self.map_height = map_msg.info.height
        self.map_width = map_msg.info.width
        self.map_resolution = map_msg.info.resolution # meters / cell
        # rospy.logerr("map dims: %s %s", self.map_height, self.map_width)
        # self.map_origin = 

    def odom_cb(self, msg):
        # print("setting odom")
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y) 

    def goal_cb(self, msg):
        print("setting goal")
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y) 

        goal_cell = self.world_to_cell(self.goal_pose)
        self.acceptable_goal_cells = {goal_cell, (goal_cell[0], goal_cell[1]+1), (goal_cell[0], goal_cell[1]-1), (goal_cell[0]+1, goal_cell[1]), (goal_cell[0]-1, goal_cell[1])}


    def cell_to_world(self, u, v):
        '''
        convert __ to __
        '''
        angle = self.map_origin[2]
        rotation_matrix = np.array([[np.cos(angle), np.sin(angle), 0], 
                                    [-np.sin(angle), np.cos(angle), 0], 
                                    [0, 0, 1]])
        rotated_coord =  np.matmul(rotation_matrix, np.array([u * self.map_resolution - self.map_origin[0], v * self.map_resolution - self.map_origin[1], 0]))
        
        return (rotated_coord[0], rotated_coord[1])

    def world_to_cell(self, position):
        angle = self.map_origin[2]
        rotation_matrix_inv = np.linalg.inv(np.array([[np.cos(angle), np.sin(angle), 0], 
                                                    [-np.sin(angle), np.cos(angle), 0], 
                                                    [0, 0, 1]]))
        orig_coord = 1/self.map_resolution * (np.matmul(rotation_matrix_inv, np.array([position[0], position[1], 0])) + self.map_origin)

        return tuple(orig_coord) # in form of (u, v)

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
            v, u = np.random.randint(0, self.map_height), np.random.randint(0, self.map_width)

            # v, u = rand_indices[0], rand_indices[1] #y direction, x direction

            if self.point_collision_check(v, u) == True: # collision happens
                continue

            else:
                return self.cell_to_world(u, v)
            

    def point_collision_check(self, v, u):
        # return True if collision exists at this cell
        return self.map[v][u] != 0

    def path_collision_check(self, start, end):
        # check that the path between start, end is collision free - identify occupancy grid squares affected and check each
        # return True if collision exists 
        checked_cells = set()

        x_orig = (start[0], end[0])
        y_orig = (start[1], end[1])

        dist = np.linalg.norm(np.array((5,5))-np.array((0,0)))
        step_dist = .5 * self.map_resolution # tune this
        num_pts = int(dist / step_dist) # num pts to interpolate
        x_interp = np.linspace(x_orig[0], y_orig[0], num_pts)
        y_interp = np.interp(x_interp, x_orig, y_orig)

        for i in range(num_pts):
            pt = self.world_to_cell((x_interp[i], y_interp[i])) # get map frame coord
            cell = (int(pt[0]), int(pt[1])) # convert to cell ind
            # print(pt)
            if cell in checked_cells:
                continue
            
            checked_cells.add(cell)
            if self.point_collision_check(cell[1], cell[0]):
                return True
            
        return False
            

    def find_nearest_vertex(self, position):
        # print("finding nearest vertex")
        # find nearest vertex to a given position
        # simplest heuristic: euclidean distance
        # could consider others like spline? dubins path? -- this can be an optimization task
        # iterate through node list and identify the one with lowest distance
        dists = np.array([np.linalg.norm(np.array(position) - np.array(v)) for v in self.parents.keys()]) # euclidean distance to all vertices
        min_ind = np.argmin(dists)
        return self.parents.keys()[min_ind]

    def reached_goal(self, node):
        # print("checking if goal reachable")
        # if can go from node to end w/o intersecting wall
        # can get rid of this function; replaced with path_collision_check(self, node, end_point) in plan_path()
        # node_cell = self.world_to_cell(node)

        # print("current node:", node_cell)

        # return node_cell in self.acceptable_goal_cells
        pass
    

    ### rrt alg ###
    def plan_path(self, start_point, end_point, map, max_distance):
        print("planning path now")
        #TODO Add max_distance parameter, new nodes should not exceed a certain distance from their nearest node
        ## CODE FOR PATH PLANNING ##
        goal_reached = False
        max_iter = 10000
        current_iter = 0

        self.parents[start_point] = None
        
        while not goal_reached and current_iter < max_iter :

            # print("current iter:", current_iter)
            node_new = self.sample_map() # point collision check is perfomed in sampling (only return valid samples)
            node_nearest = self.find_nearest_vertex(node_new)

            if self.path_collision_check(node_new, node_nearest):
                continue

            # update tree
            # self.tree[node_new] = set() # initialize new node in tree
            # self.tree[node_nearest].add(node_new) # add new node to child set of node_nearest
            self.parents[node_new] = node_nearest

            if current_iter == max_iter - 1 or not self.path_collision_check(node_new, end_point):
                goal_reached = True
                self.parents[end_point] = node_new # connect it to the goal
                break

            current_iter += 1

        
        print("path search end, iterations:", current_iter)
        # print("tree", self.parents)

        # by this point, the tree has been constructed
        # reconstruct trajectory given graph by recursing thru self.parents
        reverse_path = [end_point] # points along traj in reverse order

        current_node = end_point
        while self.parents[current_node] != None:
            current_node = self.parents[current_node] # backtrack to parent node
            reverse_path.append(current_node)

        for pt in reverse_path[::-1]: # populate trajectory object
            point_obj = Point(x=pt[0], y=pt[1])
            self.trajectory.addPoint(point_obj)

        # self.trajectory.addPoint(Point(x=0,y=0))
        # self.trajectory.addPoint(Point(x=5,y=0))
        # self.trajectory.addPoint(Point(x=0,y=2))
                    

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    
    while pf.map == None or pf.goal_pose == None or pf.current_pose == None:
        pass
    print("these are map dims:", pf.map_height, pf.map_width)
    pf.plan_path(pf.current_pose, pf.goal_pose, pf.map, None)

    # print("map origin:", pf.map_origin)
    # print("transformed origin:", pf.cell_to_world(0, 0))


    rospy.spin()
