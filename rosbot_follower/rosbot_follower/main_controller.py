import rclpy
from rclpy import duration
import rclpy.time
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import tf2_geometry_msgs

from collections import deque
from collections import OrderedDict

import numpy as np
import threading
import time

WAYPOINT_COUNT = 20

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        #subscribers
        self.camera_sub = self.create_subscription(Float64MultiArray, 'follow/camera_angle', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(Float64MultiArray, 'follow/lidar_angle', self.lidar_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/follow/scan_repeat', self.scan_callback, 10) #change scan to follow/scan later

        #publishers
        self.angle_pub = self.create_publisher(Float64, 'follow/main_angle', 10)
        self.move_pub = self.create_publisher(Bool, 'follow/nav_move', 10)
        self.hazard_pub = self.create_publisher(Marker, '/hazards', 10)
        self.hazard_array_pub = self.create_publisher(MarkerArray, '/hazards_array', 10)

        
        self.lastCamReading = self.get_clock().now()
        self.lastLidarReading = self.get_clock().now()

        self.waypoints = deque(maxlen=WAYPOINT_COUNT)
        self.scan_dict = OrderedDict()
        self.angle = (0,0)

        # Transform listener
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)

        self.waypoint_thread = threading.Thread(target=self.save_waypoints)
        self.waypoint_thread.start() #will this work here?

        self.get_logger().info("***************main controller launched********************")


    def scan_callback(self, msg):
        DICT_LIMIT = 30
        timeStamp = msg.header.stamp
        
        # Round to single decimal place for uniformity
        keyTime = round(timeStamp.sec + timeStamp.nanosec/1000000000, 1) # convert nano to seconds, 
        
        self.scan_dict[keyTime] = msg.ranges
        #self.get_logger().info(f"Saved timestamp: {keyTime} to scan dict with ranges length {len(msg.ranges)}")
        # pop oldest entry if dict gets too large
        if len(self.scan_dict) > DICT_LIMIT:
            item = self.scan_dict.popitem(last=False) 
    
    def camera_callback(self, msg):
        # self.get_logger().info(f"main controller receiving camera angle and sending it on")
        self.publish_angle(msg)
        self.lastCamReading = self.get_clock().now()
        self.get_logger().info(f"cam angle at {self.lastCamReading}\nangle: {msg.data}")

    def lidar_callback(self, msg):
        timeSinceCam = self.get_clock().now() - self.lastCamReading
        self.get_logger().info(f"lidar data recieved")
        if timeSinceCam.nanoseconds > duration.Duration(seconds=0.5).nanoseconds:
            self.get_logger().info(f"camera angle timed out, time since last msg: {timeSinceCam}\nangle: {msg.data[1]}")
            self.publish_angle(msg)
   
        self.lastLidarReading = self.get_clock().now()

    def publish_angle(self, msg):
        TIME, ANGLE = 0, 1
        self.angle = (msg.data[TIME], msg.data[ANGLE])

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

    def get_laserscan(self, timestamp):        
        #self.get_logger().info(f"Check timestep: {timestamp}")
        return self.scan_dict.get(timestamp,[])
     
    def save_waypoints(self):
        self.get_logger().info("------------------------------waypoint thread running-----------------------------------------")
        max_timeout = duration.Duration(seconds=5.0).nanoseconds
        
        TIME, ANGLE = 0,1
        while True:
            angle = self.angle[ANGLE]
            ranges = self.get_laserscan(self.angle[TIME])

            if(len(ranges) > 0):
                stamp = self.get_clock().now() #replace with proper time stamp
                dist = self.get_distance_from_laser_scan(ranges, angle)
                pose = self.generate_origin_pose()
                rad_angle = angle * (np.pi / 180)
                pose.position.x = dist * np.cos(rad_angle)
                pose.position.y = dist * np.sin(rad_angle)
                map_pose = self.transform_pose('base_link', 'map', pose, stamp) #assuming this updates the stamp in map_pose too
                self.waypoints.append(map_pose)
                self.get_logger().info(f"\nWAYPOINT:\ndistance calculated:{dist}\npose: x: {pose.position.x} y: {pose.position.y}\nmap pose: x: {map_pose.position.x} y: {map_pose.position.y}")
                # self.publish_marker_pos(map_pose)
                self.publish_hazard_array()
            
            interval = 0
            while interval < 1:
                timeSinceCam = self.get_clock().now() - self.lastCamReading
                timeSinceLidar = self.get_clock().now() - self.lastLidarReading
                if timeSinceCam.nanoseconds > max_timeout and timeSinceLidar.nanoseconds > max_timeout:
                    self.get_logger().info(f"both timed out\ntime since last cam: {timeSinceCam}\ntime since last lidar{timeSinceLidar}")
                    self.predict_waypoint()
                sleepTime = 0.5
                interval += sleepTime
                time.sleep(sleepTime)     

    def predict_waypoint(self):
        msg = Bool()
        msg.data = False
        self.move_pub.publish(msg)
        self.get_logger().info("calling nav2-----------------------------------------------------------------")

        time.sleep(10) #later replace with prediction and waiting for nav2 to finish
        
        self.get_logger().info("finished with nav2-----------------------------------------------------------")
        msg.data = True
        self.move_pub.publish(msg)
        

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
    