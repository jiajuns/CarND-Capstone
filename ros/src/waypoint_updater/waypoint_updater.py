#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()
		
		shortest_dist = 65535
		nearest_waypoint_idx = 0
		r = rospy.Rate(50) # 50hz sampling rate
		while not rospy.is_shutdown():
		    self.loop()
		
    def loop(self):
	    # step 1. find out the nearest waypoint to the current position
		# current x & y coordinates. Shall we include z???
		current_pose_x = self.current_pose.pose.position.x
		current_pose_y = self.current_pose.pose.position.y 
		# for each waypoint of the base_waypoints, calculate the distance from the current position, find out the nearest waypoint index
		for i in range(len(self.base_waypoints)):
		    # base waypoint x & y coordinates. Shall we include z???
		    base_waypoint_x = self.base_waypoints[i].pose.pose.position.x
			base_waypoint_y = self.base_waypoints[i].pose.pose.position.y
			distance = math.sqrt((current_pose_x - base_waypoint_x)**2 + (current_pose_y - base_waypoint_y)**2)
			if distance < shortest_distance:
			    shortest_distance = distance
			    nearest_waypoint_idx = i
	
	    # step 2. TODO: the nearest waypoint might be behind the car, we need to check if the nearest waypoint is at the current heading direction. We need to utilize the orientation info from the PoseStampd message
		
		# step 3. TODO: append the 200 LOOKAHEAD waypoints for the final_waypoints_pub	
	
	    r.sleep()
	

    def pose_cb(self, msg):
        # TODO: Implement
		'''msg type geometry_msgs/PoseStamped
		   geometry_msgs/Pose pose 
		     geometry_msgs/Point position
		       float64 x
			   float64 y 
			   float64 z 
		     geometry_msgs/Quaternion orientation
			   float64 x 
			   float64 y 
			   float64 z 
			   float64 w 
		'''
		self.current_pose = msg
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
		'''waypoints message type styx_msgs/Lane
           styx_msgs/Waypoint[] waypoints
             geometry_msgs/PoseStamped pose
               std_msgs/Header header
                 uint32 seq
                 time stamp
                 string frame_id
               geometry_msgs/Pose pose
                 geometry_msgs/Point position
                   float64 x
                   float64 y
                   float64 z
                 geometry_msgs/Quaternion orientation
                   float64 x
                   float64 y
                   float64 z
                   float64 w
               geometry_msgs/TwistStamped twist
                 std_msgs/Header header
                   uint32 seq
                   time stamp
                   string frame_id
                 geometry_msgs/Twist twist
                   geometry_msgs/Vector3 linear
                     float64 x
                     float64 y
                     float64 z
                   geometry_msgs/Vector3 angular
                     float64 x
                     float64 y
                     float64 z
		'''
		# get the waypoint list from the Lane message
		self.base_waypoints = waypoints.waypoints
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
