#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

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
NORMAL_DECEL = 4 # m/s^2
MAX_DECEL = 9.5 # m/2^2
NORMAL_ACCEL = 6 # m/s^2
VELOCITY_30MPH = 13.3 # m/s
REFRESH_RATE = 50 # Hz

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        self.current_pose = None
        self.base_waypoints = None
        self.stop_waypoint_idx = None #750 #286
		#self.stopped_time = 0.0

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Waypoint, self.obstacle_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        #rospy.spin()
        self.rate = rospy.Rate(REFRESH_RATE) # 50hz sampling rate
        while not rospy.is_shutdown():
            # rospy.loginfo("WaypointUpdater goes to loop")
            self.loop()
            # rospy.loginfo("Vehicle stopped time: %d", self.stopped_time)
            # if self.stopped_time >= 10: # vehicle has stopped for over 10 seconds
                # self.stop_waypoint_idx += 400
                # self.stopped_time = 0.0				

    def loop(self):
        if (self.current_pose is None) or (self.base_waypoints is None):
            return
        # step 1. find out the nearest waypoint to the current position
        # current x & y coordinates. Shall we include z???
        current_pose_x = self.current_pose.pose.position.x
        current_pose_y = self.current_pose.pose.position.y
        current_pose_z = self.current_pose.pose.position.z
        shortest_distance = +np.inf
        nearest_waypoint_idx = 0
        roll, pitch, yaw = quaternion_to_euler_angle(self.current_pose.pose.orientation.w,
                                                     self.current_pose.pose.orientation.x,
                                                     self.current_pose.pose.orientation.y,
                                                     self.current_pose.pose.orientation.z)

        # for each waypoint of the base_waypoints, calculate the distance from the current position, find out the nearest waypoint index
        for i in range(len(self.base_waypoints)):
            # base waypoint x & y coordinates.
            base_waypoint_x = self.base_waypoints[i].pose.pose.position.x
            base_waypoint_y = self.base_waypoints[i].pose.pose.position.y
            base_waypoint_z = self.base_waypoints[i].pose.pose.position.z
            distance = np.sqrt((current_pose_x - base_waypoint_x)**2 + (current_pose_y - base_waypoint_y)**2 + (current_pose_z - base_waypoint_z)**2)
            if distance < shortest_distance:
                shortest_distance = distance
                nearest_waypoint_idx = i
        # rospy.loginfo("nearest waypoint index is %d", nearest_waypoint_idx)

        # step 2. the nearest waypoint might be behind the car, we need to check if the nearest waypoint is at the current heading direction. We need to utilize the orientation info from the PoseStampd message
        nearest_waypoint_x = self.base_waypoints[nearest_waypoint_idx].pose.pose.position.x
        nearest_waypoint_y = self.base_waypoints[nearest_waypoint_idx].pose.pose.position.y
        wp_yaw = np.arctan2((nearest_waypoint_y - current_pose_y), (nearest_waypoint_x - current_pose_x)) # I`m not too sure about this part

        # calculate the angle between car's yaw and wp_yaw, only accept the waypoint if the angle is less than 45 degree, otherwise, use the next waypoint as the first lookahead waypoint. Then append the next 200 base waypoints as the lookahead waypoints. Rollover to the first base waypoint when the loop reaches the end of the base waypoint list.
        theta = yaw - wp_yaw
        lookahead_waypoints = []
        if abs(theta) < np.pi/2:
            for i in range(LOOKAHEAD_WPS):
                waypoint_idx = (nearest_waypoint_idx + i) % len(self.base_waypoints)
                lookahead_waypoints.append(self.base_waypoints[waypoint_idx])
        else:
            for i in range(LOOKAHEAD_WPS):
                waypoint_idx = (nearest_waypoint_idx + 1 + i) % len(self.base_waypoints)
                lookahead_waypoints.append(self.base_waypoints[waypoint_idx])
        
        # step 3.
        if self.stop_waypoint_idx is not None:
            if self.stop_waypoint_idx == -1:
                # no red light detected, adjust current velocity to 30MPH
                # calculate the distance the vehicle needs to travel from current velocity to 30mph
                # d=(vc^2-vo^2)/2a
                dist_to_30mph = (VELOCITY_30MPH**2 - self.current_velocity**2) / (2*NORMAL_ACCEL)
                accel_per_dist = (VELOCITY_30MPH - self.current_velocity) / (dist_to_30mph + 1e-12)
                # update the velocity of the lookahead_waypoints				
                for i in range(nearest_waypoint_idx, nearest_waypoint_idx+LOOKAHEAD_WPS):
                    dist_curr_to_i = self.distance(self.base_waypoints, nearest_waypoint_idx, i+1)
                    increased_v = dist_curr_to_i * accel_per_dist
                    velocity_i = self.current_velocity + increased_v
                    velocity_i = velocity_i if velocity_i < VELOCITY_30MPH else VELOCITY_30MPH
                    self.set_waypoint_velocity(lookahead_waypoints, i-nearest_waypoint_idx, velocity_i)
            else:
                rospy.loginfo("stop_waypoint_idx is %d", self.stop_waypoint_idx)
                # red light detected			
                # calculate the normal braking distance from the current_velocity
                # a=(vc-v0)/t, d=((vc+v0)/2)*t, v0=0  --> d=vc^2/(2*a)
                normal_brake_dist = (self.current_velocity**2)/(2*NORMAL_DECEL)
                # calculate the distance between the current position and the red light stop position. use the nearest waypoint as the current position
                dist_to_stop = self.distance(self.base_waypoints, nearest_waypoint_idx, self.stop_waypoint_idx)
                # if the car is getting close to the red light, start braking, otherwise, keep constant speed
                if dist_to_stop <= normal_brake_dist and dist_to_stop > 4:
                    rospy.loginfo(self.current_velocity)
                    decel_per_dist = self.current_velocity / (dist_to_stop + 1e-12) #* 2 # provide a factor of 1.5 to be safe
                    for i in range(nearest_waypoint_idx, self.stop_waypoint_idx):
                        dist_curr_to_i = self.distance(self.base_waypoints, nearest_waypoint_idx, i+1)
                        reduced_v = dist_curr_to_i * decel_per_dist
                        velocity_i = self.current_velocity - reduced_v
                        velocity_i = velocity_i if velocity_i > 0 else 0.0
                        if i < nearest_waypoint_idx + LOOKAHEAD_WPS:
                            self.set_waypoint_velocity(lookahead_waypoints, i-nearest_waypoint_idx, velocity_i)
                elif dist_to_stop <= 4:
                    for i in range(nearest_waypoint_idx, self.stop_waypoint_idx):
                        if i < nearest_waypoint_idx + LOOKAHEAD_WPS:
                            self.set_waypoint_velocity(lookahead_waypoints, i-nearest_waypoint_idx, 0.0)
                # adjust velocity up to 30mph if current velocity is slow and vehicle is still away from red light
                elif dist_to_stop > 10 and dist_to_stop > 2*normal_brake_dist and self.current_velocity < VELOCITY_30MPH: 
                    # calculate the distance the vehicle needs to travel from current velocity to 30mph
                    # d=(vc^2-vo^2)/2a
                    dist_to_30mph = (VELOCITY_30MPH**2 - self.current_velocity**2) / (2*NORMAL_ACCEL)
                    accel_per_dist = (VELOCITY_30MPH - self.current_velocity) / (dist_to_30mph + 1e-12)
                    # update the velocity of the lookahead_waypoints				
                    for i in range(nearest_waypoint_idx, nearest_waypoint_idx+LOOKAHEAD_WPS):
                        dist_curr_to_i = self.distance(self.base_waypoints, nearest_waypoint_idx, i+1)
                        increased_v = dist_curr_to_i * accel_per_dist
                        velocity_i = self.current_velocity + increased_v
                        velocity_i = velocity_i if velocity_i < VELOCITY_30MPH else VELOCITY_30MPH
                        self.set_waypoint_velocity(lookahead_waypoints, i-nearest_waypoint_idx, velocity_i)

            # rospy.loginfo("current_velocity: %f", self.current_velocity)
            # if self.current_velocity <= 1.0:
                # self.stopped_time = self.stopped_time + 0.02 #1/REFRESH_RATE

            # if dist_to_stop <= normal_brake_dist:
            #     decel = (self.current_velocity**2)/(2*dist_to_stop + 1e-12)
            #     if decel > MAX_DECEL:
            #         decel = MAX_DECEL
            #     # calculate the velocity for each waypoint between the current position and red light stop line
            #     for i in range(nearest_waypoint_idx, self.stop_waypoint_idx+1):
            #         dist_curr_to_i = self.distance(self.base_waypoints, nearest_waypoint_idx, i)
            #         # vi = sqrt(vc^2-2*a*d)
            #         velocity_i = np.sqrt(self.current_velocity**2 - 2*decel*dist_curr_to_i)
            #         # set velocity for each waypoint in the lookahead_waypoints
            #         if i < nearest_waypoint_idx + LOOKAHEAD_WPS:
            #             self.set_waypoint_velocity(lookahead_waypoints, i-nearest_waypoint_idx, velocity_i)
            #             if i == 0:
            #                 rospy.loginfo(velocity_i)

        # rospy.loginfo(nearest_waypoint_idx)
        # create an empty Lane message to hold the lookahead_waypoints
        lane = Lane()
        lane.waypoints = lookahead_waypoints
        self.final_waypoints_pub.publish(lane)

        self.rate.sleep()


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
        #pass

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
        #pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stop_waypoint_idx = msg.data
        #pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def velocity_cb(self, msg):
        '''msg type geometry_msgs/TwistStamped
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
        # get the vehicle's current velocity from the simulator
        self.current_velocity = msg.twist.linear.x
        #pass

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

def quaternion_to_euler_angle(w, x, y, z):
    """
    helper function to convert quaternion to euler angle
    """
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1)) #roll

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2)) #pitch

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4)) #yaw

    return X, Y, Z

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
