import rclpy
from rclpy import duration
import rclpy.time
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import tf2_geometry_msgs

from collections import deque
import numpy as np
import threading
import time

WAYPOINT_COUNT = 20

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        #subscribers
        self.camera_sub = self.create_subscription(Float64, 'follow/camera_angle', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(Float64, 'follow/lidar_angle', self.lidar_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/follow/scan_repeat', self.scan_callback, 10) #change scan to follow/scan later

        #publishers
        self.angle_pub = self.create_publisher(Float64, 'follow/main_angle', 10)
        self.hazard_pub = self.create_publisher(Marker, '/hazards', 10)
        self.hazard_array_pub = self.create_publisher(MarkerArray, '/hazards_array', 10)

        
        self.lastCamReading = rclpy.time.Time()
        self.waypoints = deque(maxlen=WAYPOINT_COUNT)
        self.laser_scan = LaserScan()
        self.angle = 0

        # Transform listener
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)

        self.waypoint_thread = threading.Thread(target=self.save_waypoints)
        self.waypoint_thread.start() #will this work here?

        self.get_logger().info("***************main controller launched********************")


    def scan_callback(self, msg):
        self.laser_scan = msg
    
    def camera_callback(self, msg):
        # self.get_logger().info(f"main controller receiving camera angle and sending it on")
        self.angle = msg.data
        self.angle_pub.publish(msg)
        self.lastCamReading = self.get_clock().now()

    def lidar_callback(self, msg):
        #timeSinceCam = self.get_clock().now() - self.lastCamReading
        #if timeSinceCam > duration.Duration(seconds=0.5):
        #    self.get_logger().info(f"camera angle timed out, time since last msg: {timeSinceCam}")
        self.angle_pub.publish(msg)        
    
    def get_distance_from_laser_scan(self, ranges, angle):
        index = int(angle)*2
        distance = ranges[index]

        #if the laser cuts out for the required angle, just look to the side a little bit
        oldIndex = index
        while distance == np.inf:
            index += 1
            if index == len(ranges):
                index = 0
            if index == oldIndex: #to make sure we don't get stuck here forever
                break
            distance = ranges[index]

        # self.get_logger().info(f"\nDISTANCE CALCULATION:*************************************\nangle: {angle}\ndistance at angle: {distance}\nindex used: {index}\nindex expected: {angle * 2}")
        return distance
    
    def generate_origin_pose(self):
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        return pose
    
    def transform_pose(self, src_frame, dest_frame, pose, time):
        time = time - rclpy.duration.Duration(seconds=0.1)
        for _ in range (1, 6):
            try:
                timeout = rclpy.duration.Duration(seconds=0.05)
                transform = self.tf_buffer.lookup_transform(dest_frame, src_frame, time, timeout=timeout)
                poseT = tf2_geometry_msgs.do_transform_pose(pose, transform)
                return poseT
            except tf2_ros.TransformException as ex:
                time = time - rclpy.duration.Duration(seconds=1)

        return self.generate_origin_pose()
    
    #for debuggin
    def generate_marker(self, pose, id):
        time = self.get_clock().now()
        
        # Create the marker
        marker_msg = Marker()
        marker_msg.header.frame_id = 'map'
        marker_msg.header.stamp = time.to_msg()
        marker_msg.type = Marker.SPHERE 
        marker_msg.action = Marker.ADD
       
        marker_msg.id = id #assuming this is how we publish id 
        marker_msg.pose = pose
        
        marker_msg.scale.x = 0.1  
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.1
        marker_msg.color.r = 0.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 0.1 * id
        marker_msg.color.a = 1.0
        
        # Infinite lifetime
        marker_msg.lifetime.sec = 30
        return marker_msg

    def publish_marker_pos(self, pose):
        marker = self.generate_marker(pose, 1)

        # self.get_logger().info(f'PUBLISHING MARKER {marker_msg.pose}')
        # Publish
        self.hazard_pub.publish(marker)

    def publish_hazard_array(self):
        array = MarkerArray()
        for i, pose in enumerate(self.waypoints):
            marker = self.generate_marker(pose, i+1)
            array.markers.append(marker)
        self.hazard_array_pub.publish(array)

    
    def save_waypoints(self):
        self.get_logger().info("------------------------------waypoint thread running-----------------------------------------")
        while True:
            ranges = self.laser_scan.ranges #replace with getting ranges at specific time stamp
            angle = self.angle
            if(len(ranges) > 0):
                stamp = self.get_clock().now() #replace with proper time stamp
                dist = self.get_distance_from_laser_scan(ranges, angle)
                pose = self.generate_origin_pose()
                rad_angle = angle * (np.pi / 180)
                pose.position.x = dist * np.cos(rad_angle)
                pose.position.y = dist * np.sin(rad_angle)
                map_pose = self.transform_pose('base_link', 'map', pose, stamp) #assuming this updates the stamp in map_pose too
                self.waypoints.append(map_pose)
                # self.get_logger().info(f"\nWAYPOINT:\ndistance calculated:{dist}\npose: x: {pose.position.x} y: {pose.position.y}\nmap pose: x: {map_pose.position.x} y: {map_pose.position.y}")
                # self.publish_marker_pos(map_pose)
                self.publish_hazard_array()
                time.sleep(1)
        
    

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    main_controller = MainController()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(main_controller)
    main_controller.waypoint_thread.join() #this will probably never join
    # Explicity destroy the node
    main_controller.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    