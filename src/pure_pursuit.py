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
from std_msgs.msg import Header

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic", "/pf/pose/odom")
        # self.lookahead        = # FILL IN #
        self.speed            = 2
        # self.wheelbase_length = # FILL IN #
        
        self.pose = None

        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.pose_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)

        self.min_dist_pub = rospy.Publisher("/min_point", PointStamped, queue_size=1)

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

    def pose_callback(self, odom):
        self.pose = odom.pose.pose
        min_point, dist = self.min_dist([self.pose.position.x, self.pose.position.y], self.trajectory.points[0], self.trajectory.points[1])
        self.min_dist_pub.publish(PointStamped(point=Point(x=min_point[0], y=min_point[1], z=0), header=Header(frame_id="map")))

    def min_dist(self, robot_position, v, w):
        # Return minimum distance between line segment vw and point p
        # v, w are Point
        robot_position = np.array(robot_position)
        w = np.array(w)
        v = np.array(v)

        l2 = np.linalg.norm(w-v)**2  # i.e. |w-v|^2 -  avoid a sqrt
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
    rospy.spin()
