#source:
#group 13 snc (and therefore all of its sources)
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float32MultiArray, Float64
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs

import tf2_ros

import numpy as np
import cv2

CAMERA_FIELD_OF_VIEW = 120 #still not sure about this one
IMAGE_WIDTH = 640

class Tracker(Node):

    def __init__(self):
        super().__init__('mapper')
        self.reentrant_group_1 = ReentrantCallbackGroup()

        #subscribers
        self.object_sub = self.create_subscription(Float32MultiArray, '/objects', self.object_callback,
            10, callback_group=self.reentrant_group_1)
        
        # Publishers
        self.robot_pose_pub = self.create_publisher(PoseStamped, '/follow/pose', 10)
        self.angular_offset_pub = self.create_publisher(Float64, '/follow/angular_gain', 10)

        # Transform listener
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)

        # Flags
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
    
    def calculate_pose_from_object(self, object_msg_data):

        dst_pts = self.dst_points(object_msg_data) 

        #in image coordinates
        centre_x, _ = self.get_centre_index(dst_pts)
        msg = Float64()
        msg.data = float(centre_x)
        
        self.get_logger().info(f"tracker sending angular offset {msg}")
        self.angular_offset_pub.publish(msg)
        
    def object_callback(self, object_msg):
        data = object_msg.data
        if len(data) > 0:
            self.calculate_pose_from_object(data)

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