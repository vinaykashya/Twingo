#!/usr/bin/env python

import rospy
import math
import tf
from   geometry_msgs.msg import PoseStamped, TwistStamped
from   styx_msgs.msg     import Lane, Waypoint
from   std_msgs.msg      import Int32, Float32
from   copy              import deepcopy


LOOKAHEAD_WPS = 200
MAX_DECEL     = 4.0
STOP_BUFFER   = 5.0


class WaypointUpdater(object):

    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose',      PoseStamped, self.current_pose_cb)
        rospy.Subscriber('/base_waypoints',    Lane,        self.base_waypoints_cb)
        rospy.Subscriber('/current_velocity',  TwistStamped,self.current_velocity_cb)
	#TODO - Subscribe to traffic light and obstacle detectors

        self.current_velocity = 0.0
        self.decel = 1.0
        self.traffic_waypoint = -1
        self.braking = False

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


    def loop(self):
        if hasattr(self, 'base_waypoints') and hasattr(self, 'current_pose'):
            lane                 = Lane()
            lane.header.stamp    = rospy.Time().now()
            lane.header.frame_id = '/world'

            current_pose = self.current_pose
            waypoints = self.base_waypoints.waypoints

            next_waypoint_idx    = self.waypoint_idx_ahead_of_car(current_pose, waypoints)
	    #TODO - obtain traffic_light_waypoint


	    #TODO - 1. Check if initializing (No traffic lights or obstacles)
	    #TODO - 2. Check if we are braking (To stop at TL)
	    #TODO - 3. If braking for TL, final way points will be (next_waypoint+ dist to TL)
	    #TODO - Default - final way points will be (next_waypoint+ LOOKAHEAD)
            lane.waypoints = self.next_waypoints_set(waypoints, next_waypoint_idx, next_waypoint_idx+LOOKAHEAD_WPS)
            self.final_waypoints_pub.publish(lane)

    def waypoint_idx_ahead_of_car(self, pose, waypoints):
        min_distance = float('inf')
        closest_waypoint_idx = 0
        for i in range(len(waypoints)):
            dist = self.distance(pose.pose.position, waypoints[i].pose.pose.position)
            if dist < min_distance:
                min_distance = dist
                closest_waypoint_idx = i
        waypoint_x = waypoints[closest_waypoint_idx].pose.pose.position.x
        waypoint_y = waypoints[closest_waypoint_idx].pose.pose.position.y
        heading = math.atan2( (waypoint_y-pose.pose.position.y), (waypoint_x-pose.pose.position.x) )
        tform = tf.transformations.euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        angle = math.fabs(tform[-1]-heading)
        if angle > math.pi / 4.0:
            closest_waypoint_idx += 1
        return closest_waypoint_idx

    def next_waypoints_set(self, waypoints, start_wp, end_wp):
        final_waypoints = []
        for i in range(start_wp, end_wp):
            index = i % len(waypoints)
            wp = Waypoint()
            wp.pose.pose.position.x  = waypoints[index].pose.pose.position.x
            wp.pose.pose.position.y  = waypoints[index].pose.pose.position.y
            wp.pose.pose.position.z  = waypoints[index].pose.pose.position.z
            wp.pose.pose.orientation = waypoints[index].pose.pose.orientation
	    wp.twist.twist.linear.x = waypoints[index].twist.twist.linear.x
	    #TODO - if braking, set twist to a small value 
	    #TODO - if at TL, twist should be 0
	    #default
	    wp.twist.twist.linear.x = waypoints[index].twist.twist.linear.x
            final_waypoints.append(wp)
	    #TODO -Decelerate while braking if TL is detected
        return final_waypoints


    def distance(self, p1, p2):
        x = p1.x - p2.x
        y = p1.y - p2.y
        z = p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)


    def current_pose_cb(self, msg):
        self.current_pose = msg


    def base_waypoints_cb(self, msg):
        self.base_waypoints = msg


    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x


    #TODO - callback for obstacle and traffic light detector subscribers






if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
