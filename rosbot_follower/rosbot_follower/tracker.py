#source:
#group 13 snc (and therefore all of its sources)
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time


from std_msgs.msg import Float32MultiArray, Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose
import tf2_geometry_msgs

import tf2_ros
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler

import numpy as np
import cv2
#listen to find object 2d to get
#when something comes through, calculate distance using laser
#calculate target position from that and publish

CAMERA_FIELD_OF_VIEW = 120 #still not sure about this one
IMAGE_WIDTH = 640

class Tracker(Node):
    def __init__(self):
        super().__init__('mapper')
        self.reentrant_group_1 = ReentrantCallbackGroup()

        #subscribers
        self.object_sub = self.create_subscription(Float32MultiArray, '/objects', self.object_callback,
            10, callback_group=self.reentrant_group_1)
        
        self.laser_scan_sub = self.create_subscription(LaserScan, '/scan', self.laser_scan_callback,
            10, callback_group=self.reentrant_group_1)
        
        #publishers
        self.robot_pose_pub = self.create_publisher(PoseStamped, '/follow/pose', 10)
        self.angular_offset_pub = self.create_publisher(Float64, '/follow/angular_gain', 10)

        self.laser_scan = LaserScan()

         # Transform listener
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)

        #flags
        self.whole_target_pos = False



    def dst_points(self, detected_object_lst):
        object_width = detected_object_lst[1]
        object_height = detected_object_lst[2]

        homography_matrix = np.zeros((3, 3), dtype='float32')

        for i in range (0, 9):
            homography_matrix[i % 3, i // 3] = detected_object_lst[i+3] 

        top_left = [0,0] #in opencv, origin is at top-left and width goes first, then height
        top_right = [object_width, 0]
        bottom_left = [0, object_height]
        bottom_right = [object_width, object_height]

        src_pts = np.array([[top_left, top_right, bottom_left, bottom_right]], dtype='float32')
        # self.get_logger().info(f"homography matrix: {homography_matrix}\nsrc_pts: {src_pts}")
        dst_pts = cv2.perspectiveTransform(src_pts, homography_matrix) 
        # self.get_logger().info(f"dst_pts: {dst_pts}")
        return dst_pts[0] #return a list of points rather than points being wrapped in two lists
    
    def get_centre_index(self, corner_points):
        x = int((corner_points[0,0] + corner_points[1,0] + corner_points[2,0] + corner_points[3,0]) // 4) 
        y = int((corner_points[0,1] + corner_points[1,1] + corner_points[2,1] + corner_points[3,1]) // 4)
        # self.get_logger().info(f'centre indexes: x: {x}, y: {y}')
        return x, y
    
    def angle_from_image_index(self, x_index):
        #centre offset should be positive if image pixel is to the left of image cenre
        self.get_logger().info(f"x_index: {x_index}")
        centre_offset = IMAGE_WIDTH / 2 - x_index
        degrees_per_pixel = CAMERA_FIELD_OF_VIEW / IMAGE_WIDTH
        angle = centre_offset * degrees_per_pixel
        self.get_logger().info(f"angle (deg): {angle}")
        return angle

    def angle_from_image_index_rad(self, x_index):
        return self.angle_from_image_index(x_index) * (np.pi / 180) 
    
    def get_distance_from_laser_scan(self, ranges, centre_x):
        angle = int(self.angle_from_image_index(centre_x))
        index = angle*2
        self.get_logger().info(f"index: {index}, ranges len: {len(ranges)}")
        
        distance = ranges[index]

        #if the laser cuts out for the required angle, just look to the side a little bit
        oldIndex = index
        while distance == np.inf:
            index += 1
            if index == len(ranges):
                index = 0
            if index == oldIndex: #to make sure we don't get stuck here forever
                raise Exception("laser is full of infinities") #come up with better solution
            distance = ranges[index]

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
        #not the most amazing implementation but usable
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
    
    def calculate_pose_from_object(self, object_msg_data):

        dst_pts = self.dst_points(object_msg_data) 

        #in image coordinates
        centre_x, _ = self.get_centre_index(dst_pts)
        msg = Float64()
        msg.data = float(centre_x)
        self.get_logger().info(f"tracker sending angular offset {msg}")
        self.angular_offset_pub.publish(msg)
        # angle = self.angle_from_image_index_rad(centre_x)
        
        # if self.whole_target_pos:
        #     laser_image = self.laser_scan

        #     object_distance = self.get_distance_from_laser_scan(laser_image.ranges, centre_x)
            
        #     #from now on laser frame coordinates, continuing as if laser frame and rgb camera frame were the same
        #     laser_frame_x = object_distance * np.cos(angle)
        #     laser_frame_y = object_distance * np.sin(angle)

        #     laser_pose = self.generate_origin_pose()

        #     # - because laser is the other way round from everything else
        #     laser_pose.position.x = - laser_frame_x
        #     laser_pose.position.y = - laser_frame_y

        #     # self.get_logger().info(f"cam frame pose: x: {laser_frame_x}, y: {laser_frame_y}")

        #     stamp = laser_image.header.stamp
        #     pose = self.transform_pose('laser', 'map', laser_pose, Time.from_msg(stamp))
        # else:
        #     rotated_pose = self.generate_origin_pose()
        #     orientation = quaternion_from_euler(0, 0, angle) #angle is around z axis so yaw
        #     rotated_pose.orientation.x = orientation[0]
        #     rotated_pose.orientation.y = orientation[1]
        #     rotated_pose.orientation.z = orientation[2]
        #     rotated_pose.orientation.w = orientation[3]

        #     time = self.get_clock().now()
        #     stamp = Time.to_msg(time)
        #     pose = self.transform_pose('camera_color_frame', 'map', rotated_pose, time)

        # pose_stamped = PoseStamped()
        # pose_stamped.pose = pose
        # pose_stamped.header.stamp = stamp
        # pose_stamped.header.frame_id = 'map'
        # return pose_stamped

    def publish_pose(self, pose):
        self.robot_pose_pub.publish(pose)
    
    def laser_scan_callback(self, scan_msg):
        self.laser_scan = scan_msg

    def object_callback(self, object_msg):
        data = object_msg.data
        if len(data) > 0:
            self.calculate_pose_from_object(data)
            # followed_robot_pose = self.calculate_pose_from_object(data)
            # self.publish_pose(followed_robot_pose)
            #instead of publishing the pose we could just call another function to navigate us here
            

def main(args=None):
    rclpy.init(args=args)
    tracker = Tracker()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(tracker)

    try:
        executor.spin()
    finally:
        tracker.destroy_node()
        rclpy.shutdown()