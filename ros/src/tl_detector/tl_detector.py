#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Point
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

STATE_COUNT_THRESHOLD = 3
WAYPOINT_THRESHOLD = 50

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.tl_wps = []
        self.light_classifier = None
		self.disable_camera = False #True

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.has_image = False

        self.drop_count = 0
        self.drop_every_num_frames = 25
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
		if self.disable_camera == True:
		    return
			
        self.drop_count += 1
        if self.drop_count == self.drop_every_num_frames:
            rospy.loginfo("Got new image!!!!")
            self.has_image = True
            self.camera_image = msg
            light_wp, state = self.process_traffic_lights()
            rospy.loginfo("light_wp = %s, state = %s", light_wp, state)
            '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''
            if self.state != state:
                self.state_count = 0
                self.state = state
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1
            self.drop_count = 0


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        idx = -1
        dis_closest = -1

        if self.waypoints is not None:
            wps = self.waypoints.waypoints
            for i in range(len(wps)):
                wp = wps[i]
                dist = (wp.pose.pose.position.x-pose.x)**2 + \
                       (wp.pose.pose.position.y-pose.y)**2
                if (dis_closest == -1) or (dis_closest > dist) :
                    dis_closest = dist
                    idx = i
        return idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        if self.light_classifier is None:
            self.light_classifier = TLClassifier()
        return self.light_classifier.get_classification(cv_image)
        #return  TLClassifier().get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_wp = None
        state = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(len(self.tl_wps) == 0):
            for stop_line in stop_line_positions:
                point = stop_line[:]
                point.append(0)
                self.tl_wps.append(self.get_closest_waypoint(Point(stop_line[0],stop_line[1],0)))

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose.position)
            #TODO find the closest visible5 traffic light (if one exists)
            nearest_diff = -1
            idx = -1
            for i in range(len(self.tl_wps)):
                diff = self.tl_wps[i] - car_position
                if (diff > 0) and ((nearest_diff == -1) or (nearest_diff > diff)):
                    nearest_diff = diff
                    idx = i
            if idx != -1 and nearest_diff < WAYPOINT_THRESHOLD:
                light = self.lights[idx]
                light_wp = self.tl_wps[idx]

        if light:
            state = self.get_light_state(light)
            # check the result
            if light.state != TrafficLight.UNKNOWN:
                if state != light.state:
                    rospy.logfatal('tl_detect error, state is %s but should be %s', state, light.state)
                    state = light.state
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
