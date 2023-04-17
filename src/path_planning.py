#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.map = None
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


    def map_cb(self, map_msg):  # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double)/100.
        self.map = np.clip(self.map, 0, 1)

        # Convert the origin to a tuple
        # origin_p = map_msg.info.origin.position
        # origin_o = map_msg.info.origin.orientation
        # origin_o = tf.transformations.euler_from_quaternion((
        #         origin_o.x,
        #         origin_o.y,
        #         origin_o.z,
        #         origin_o.w))
        # origin = (origin_p.x, origin_p.y, origin_o[2])
        pass ## REMOVE AND FILL IN ##


    def odom_cb(self, msg):
        self.current_pose = msg.pose
        pass ## REMOVE AND FILL IN ##


    def goal_cb(self, msg):
        self.goal_pose = msg.pose
        pass ## REMOVE AND FILL IN ##

    def sample_map(self, map):
        # pick next random point
        pass

    def collision_check(self, map, position):
        # return True is collision exists at this point
        pass 

    def find_nearest_vertex(self, graph, position):
        # find nearest vertex to a given position
        pass

    def plan_path(self, start_point, end_point, map):
        ## CODE FOR PATH PLANNING ##
        goal_reached = False
        max_iter = 100_000
        current_iter = 0
        while not goal_reached and current_iter <= max_iter :
            node_new = self.sample_map(map)
            if self.collision_check(map, node_new):
                continue 
            node_nearest = self.find_nearest_vertex(self.tree, node_new)

            # update tree
            self.tree[node_new] = set() # initialize new node in tree
            self.tree[node_nearest].add(node_new) # add new node to child set of node_nearest 
            self.parents[node_new] = node_nearest 

            if node_new == end_point:
                break # end while
            
            current_iter += 1
        
        # by this point, the tree has been constructed
        # need to reconstruct trajectory given graph: recurse thru self.parents

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
