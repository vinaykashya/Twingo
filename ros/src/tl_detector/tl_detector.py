#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import sys
import numpy as np
import time

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.bridge = CvBridge()

        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.Traffic_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        self.sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        self.sub2.unregister()

    def traffic_cb(self, msg):
        self.lights = msg.lights
	self.sub3.unregister()

    def image_cb(self, msg):
        self.has_image = True
        self.camera_image = msg

        light_wp, state = self.process_traffic_lights()
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD :
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.Traffic_light_pub.publish(Int32(light_wp))
        else:
            self.Traffic_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, position_x, position_y):
        closestWaypoint_idx = -1
        if self.waypoints is None:
            return closestWaypoint_idx
	closestLen = 10000
        closestWaypoint = None
        pos_x = position_x
        pos_y = position_y
        for i, waypoint in enumerate(self.waypoints):
            wp_x = waypoint.pose.pose.position.x
            wp_y = waypoint.pose.pose.position.y
            dist = math.sqrt((pos_x - wp_x)*(pos_x - wp_x) + (pos_y - wp_y)*(pos_y - wp_y))
            if (dist < closestLen):
                closestLen = dist
                closestWaypoint_idx = i
        return closestWaypoint_idx


    def project_to_image_plane(self, point_in_world_x, point_in_world_y, point_in_world_z):
        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        trans = None
        rot = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link","/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link","/world", now)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")
        if trans == None or rot == None:
            return False, -1,-1
        world_point = np.array([point_in_world_x, point_in_world_y, point_in_world_z]).reshape(1,3,1)
        K_mat = np.matrix([[fx, 0,  image_width/2],
                               [0, fy, image_height/2],
                               [0,  0,            1]])
        distCoeff = None
	x,y,z,s = rot
	rod_vec = np.matrix([[1-2.0*(y**2 + z**2), 2.0*(x*y-s*z), 2.0*(x*z+s*y)],[2.0*(x*y+s*z), 1-2.0*(x**2+z**2), 2.0*(y*z-s*x)],[2.0*(x*z-s*y), 2.0*(y*z+s*x), 1-2.0*(x**2+y**2)]])
        rot_vec, _ = cv2.Rodrigues(rod_vec) 

        ret, _ = cv2.projectPoints(world_point, rot_vec, np.array(trans).reshape(3,1), K_mat, distCoeff)
        ret = ret.reshape(2,)
        u = int(round(ret[1]))
        v = int(round(ret[0]))
        valid_light = False
        if u >= 0 and u < image_width and v >= 0 and v <= image_height:
            valid_light = True
        return  valid_light, u, v

    def get_light_state(self, light_pos_x, light_pos_y, light_pos_z):
        if(not self.has_image):
            self.prev_light_loc = None
            return False
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        valid, pos_x, pos_y = self.project_to_image_plane(light_pos_x, light_pos_y, light_pos_z)
        state = TrafficLight.UNKNOWN
        if valid:
            state = self.light_classifier.get_classification(cv_image)
        return state

    def get_nearest_traffic_light(self, start_index):
        traffic_light_3D_pos = None
        TL_Pos = self.config['stop_line_positions']
        last_index = sys.maxsize
        for i in range(0, len(TL_Pos)):
            index = self.get_closest_waypoint(float(TL_Pos[i][0]), float(TL_Pos[i][1]))
            if index > start_index and index < last_index:
                last_index = index;
		x,y,z = self.lights[i].pose.pose.position.x,self.lights[i].pose.pose.position.y,self.lights[i].pose.pose.position.z
                traffic_light_3D_pos = [x,y,z]
        return traffic_light_3D_pos, last_index


    def process_traffic_lights(self):
        if(self.pose):
            #find the closest visible traffic light (if one exists)
            car_position = self.get_closest_waypoint(self.pose.position.x, self.pose.position.y)

            light_pos = None
            if car_position > 0:
                light_pos, light_waypoint = self.get_nearest_traffic_light(car_position)
                if light_pos:
                    state = TrafficLight.UNKNOWN
                    state = self.get_light_state(light_pos[0], light_pos[1], light_pos[2] if len(light_pos) >= 3 else 0.)
                    return light_waypoint, state
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

