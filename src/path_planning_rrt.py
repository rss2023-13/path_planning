#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
import rospkg
import time, os
import tf.transformations as tf
from utils import LineTrajectory
from scipy import ndimage
from std_msgs.msg import Header
import cv2

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

        self.run_planner = True # set this to False when doing path_collision_check interpolation testing

        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)

        self.trajectory = LineTrajectory("/planned_trajectory")
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=1)

        self.point_sub = rospy.Subscriber('/clicked_point', PointStamped, self.point_cb, queue_size=1)
        self.points = PoseArray()
        self.points.header = self.trajectory.make_header("/map")
        self.vertex_pub = rospy.Publisher("/vertices", PoseArray, queue_size=1)


        # use 2 dictionaries to manage rrt graph structure / path reconstruction
        # self.tree = {} # parent : set(children) --- actually maybe don't need this, not really using it
        self.parents = {} # child : parent

        # plan path
        # while self.map == None or self.goal_pose == None or self.current_pose == None:
        #     rospy.spin()
            
        # self.plan_path(self.current_pose, self.goal_pose, self.map, None)

    def visualize_point(self, point):
        vertex_pose = Pose()
        vertex_pose.position.x = point[0]
        vertex_pose.position.y = point[1]
        self.points.poses.append(vertex_pose)

        self.vertex_pub.publish(self.points)

    #nandini's visualization
    def visualize_points(self, n, x_points, y_points):
        self.points.poses = []
        for i in range(n):
            self.points.poses.append(Pose(position = Point(x = x_points[i], y=y_points[i])))

    def point_cb(self, point_msg):
        point_coords = (point_msg.point.x, point_msg.point.y)
        # print(self.path_collision_check(point_coords, self.goal_pose))


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
        old_map = np.array(map_msg.data)
        old_map[old_map == -1] = 1 # unknown cell set to occupied
        old_map[old_map == 100] = 1

        old_map = old_map.reshape(map_msg.info.height, map_msg.info.width)
        
        kernel = np.ones((15,15), np.uint8)
        self.map = cv2.dilate(old_map.astype(np.uint8), kernel, iterations=1)
         # index map as grid[y direction, x direction]

        self.map_height = map_msg.info.height
        self.map_width = map_msg.info.width
        self.map_resolution = map_msg.info.resolution # meters / cell
        print("resolution:", self.map_resolution)
        # rospy.logerr("map dims: %s %s", self.map_height, self.map_width)
        # self.map_origin = 

    def odom_cb(self, msg):
        # print("setting odom")
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y) 

    def goal_cb(self, msg):
        # print("setting goal + running plan_path")
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y) 

        self.trajectory.clear()
        self.parents = {}

        if self.run_planner == True:
            self.plan_path(self.current_pose, self.goal_pose, max_distance = 5)

    def cell_to_world(self, u, v):
        '''convert from map frame to world frame'''
        angle = self.map_origin[2]
        rotation_matrix = np.array([[np.cos(angle), np.sin(angle), 0], 
                                    [-np.sin(angle), np.cos(angle), 0], 
                                    [0, 0, 1]])
        rotated_coord =  np.matmul(rotation_matrix, np.array([u * self.map_resolution - self.map_origin[0], v * self.map_resolution - self.map_origin[1], 0]))
        
        return (rotated_coord[0], rotated_coord[1])

    def world_to_cell(self, position):
        '''convert from world frame to cell'''
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
        # print("checking path for collision")
        # check that the path between start, end is collision free - identify occupancy grid squares affected and check each
        # Return True if there is a collision
        checked_cells = set()

        if start[0] < end[0]:
            x_orig = (start[0], end[0])
            y_orig = (start[1], end[1])
        else:
            x_orig = (end[0], start[0])
            y_orig = (end[1], start[1])

        dist = np.linalg.norm(np.array(start)-np.array(end))
        step_dist = .25 * self.map_resolution # tune this
        num_pts = int(dist / step_dist) # num pts to interpolate
        
        x_interp = np.linspace(x_orig[0], x_orig[1], num_pts)
        y_interp = np.interp(x_interp, x_orig, y_orig)

        self.visualize_points(num_pts, x_interp, y_interp)

        for i in range(num_pts):
            pt = self.world_to_cell((x_interp[i], y_interp[i])) # get map frame coord
            cell = (int(pt[0]), int(pt[1])) # convert to cell ind
            # print(pt)
            # if cell in checked_cells:
                # continue
            # else:
                # checked_cells.add(cell)

            # self.visualize_point((x_interp[i], y_interp[i]))

            if self.point_collision_check(cell[1], cell[0]):
                # print("path collision")
                return True
            
        return False
    
    def calculate_new_node(self, node_sampled, node_nearest, max_distance):
        # print("calculating new node")
        node_sampled = np.array(node_sampled)
        node_nearest = np.array(node_nearest)
        dist = np.linalg.norm(node_sampled-node_nearest)
        if dist < max_distance:   
            return tuple(node_sampled)
        elif dist >= max_distance:
            unit_vector = (node_sampled-node_nearest) / dist
            node_new = node_nearest + unit_vector * max_distance # take point that is max_distance away from node_nearest in direction of node_sampled
            return tuple(node_new)
        

    def find_nearest_vertex(self, position):
        # print("finding nearest vertex")
        # find nearest vertex to a given position
        # simplest heuristic: euclidean distance
        # could consider others like spline? dubins path? -- this can be an optimization task
        # iterate through node list and identify the one with lowest distance
        dists = np.array([np.linalg.norm(np.array(position) - np.array(vert)) for vert in self.parents.keys()]) # euclidean distance to all vertices

        # sorted_args = np.argsort(dists)

        min_ind = np.argmin(dists)

        return self.parents.keys()[min_ind]

        # existing_nodes = np.array(self.parents.keys())[sorted_args]

        # for node in existing_nodes: 
        #     if not self.path_collision_check(position, node):
        #         return tuple(node)
        print("no vertices free")
    

    ### rrt alg ###
    def plan_path(self, start_point, end_point, max_distance):
        # print("planning path now")
        #TODO Add max_distance parameter, new nodes should not exceed a certain distance from their nearest node
        ## CODE FOR PATH PLANNING ##
        goal_reached = False
        max_iter = 1000
        current_iter = 0

        self.parents[start_point] = None

        vertices = PoseArray()
        vertices.header = self.trajectory.make_header("/map")

        if not self.path_collision_check(start_point, end_point): # if there's a direct path between start and end
            self.parents[end_point] = start_point
            goal_reached = True 
        
        while not goal_reached and current_iter < max_iter :
            # print("current iter:", current_iter)
            node_sampled = self.sample_map() # point collision check is perfomed in sampling (only return valid samples)
            node_nearest = self.find_nearest_vertex(node_sampled)

            node_new = self.calculate_new_node(node_sampled, node_nearest, max_distance)

            if self.path_collision_check(node_new, node_nearest): # discard point if path not feasible
                continue

            self.parents[node_new] = node_nearest # add to tree
            # self.tree[node_new] = set() # initialize new node in tree
            # self.tree[node_nearest].add(node_new) # add new node to child set of node_nearest

            if current_iter == max_iter - 1 or (not self.path_collision_check(node_new, end_point) 
                                                and np.linalg.norm(np.array(node_new)-np.array(end_point))<max_distance):
                goal_reached = True
                self.vertex_pub.publish(self.points)
                # print('final', self.points, goal_reached, current_iter)
                self.parents[end_point] = node_new # connect it to the goal

                print("path search ended, used iterations:", current_iter)
                print("from", start_point, "to", end_point)
                break

            current_iter += 1

        # print("path search end, iterations:", current_iter)
        
        # print("tree", self.parents)

        # by this point, the tree has been constructed
        # reconstruct trajectory given graph by recursing thru self.parents
        reverse_path = [end_point] # points along traj in reverse order

        current_node = end_point
        while self.parents[current_node] != None:
            current_node = self.parents[current_node] # backtrack to parent node
            reverse_path.append(current_node)

        print("path reconstructed, is", len(reverse_path), "points long")
        for pt in reverse_path[::-1]: # populate trajectory object
            point_obj = Point(x=pt[0], y=pt[1])
            self.trajectory.addPoint(point_obj)

        # self.trajectory.addPoint(Point(x=0,y=0))
        # self.trajectory.addPoint(Point(x=1,y=0))
        # self.trajectory.addPoint(Point(x=0,y=1))
                    

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
    # pf.plan_path(pf.current_pose, pf.goal_pose, pf.map, None)

    # print("map origin:", pf.map_origin)
    # print("transformed origin:", pf.cell_to_world(0, 0))


    rospy.spin()
