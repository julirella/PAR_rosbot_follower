import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan, Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64
import numpy as np

import geometry_msgs.msg
import tf2_geometry_msgs
import tf2_ros

CAMERA_WIDTH = 640
ANGULAR_GAIN = 0.002
MIN_ANG_VEL = 0.15
MAX_ANG_VEL = 0.5
DESIRED_DIST = 0.6
MIN_LIN_VEL  = 0.05
MAX_LIN_VEL  = 0.4
LINEAR_GAIN  = 0.4

class Follow(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('follow')
        # create the publisher object
        self.cmd_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.follow_subscriber = self.create_subscription(
        #                         Bool,
        #                         '/follow',
        #                         self.start_move,
        #                         10
        #                 )
        self.anglular_offset_subscriber = self.create_subscription(Float64, '/follow/angular_gain', self.angular_move_callback, 10)
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        # self.fr_subscriber = self.create_subscription(Range, '/range/fr', self.fr_callback, 10)
        # self.fl_subscriber = self.create_subscription(Range, '/range/fl', self.fl_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        #self.subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        # define the timer period for 0.5 seconds
        #self.timer_period = 0.5
        # define the variable to save the received info
        self.fr_dst = 0
        self.fl_dst = 0
        self.laser_forward = 0
        #self.laser_left = 0
        #self.laser_rear_left = 0
        #self.laser_rear_right = 0     
        #self.facing = 0
        #self.start = True
        #self.moving = True

        #flags
        self.go = False

        # create a Twist message
        self.cmd = Twist()
        #self.timer = self.create_timer(self.timer_period, self.motion)

        # transform
        #self.tf_buffer = tf2_ros.buffer.Buffer()
        #self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)

        # Setup timer callback
        #self.timer = self.create_timer(1.0, self.transform)
    def divide_sections(self, msg):
        length = len(msg.ranges)
        each_div = length // 8
        front_start = length - each_div // 2
        front_end = each_div // 2

        sections = {
            "front": min(min(msg.ranges[front_start:]), min(msg.ranges[:front_end])),
            "fleft": min(msg.ranges[each_div // 2 : each_div]),
            "fright": min(msg.ranges[front_start - each_div // 2 : front_start]),
        }
        return sections
    
    def laser_callback(self,msg):
        # Save the frontal laser scan info at 0°
        sections = self.divide_sections(msg)
        front_dst = sections['front']

        if front_dst < DESIRED_DIST:
            self.get_logger().info(f"something is too close")
            self.cmd.linear.x = 0.0     
        else:
            self.cmd.linear.x = LINEAR_GAIN*(front_dst - DESIRED_DIST)
            self.get_logger().info(f"ready to go, linear velocity: {self.cmd.linear.x}")


        # self.get_logger().info(f"publishing cmd_vel: {self.cmd}")
        # self.cmd_publisher_.publish(self.cmd)  
         
    # def start_move(self, data):
    #     if data:
    #         self.cmd.linear.x = 0.5     
    #         self.publisher_.publish(self.cmd)

    
    def fr_callback(self, msg):
        self.fr_dst = msg.range

    def fl_callback(self, msg):
        self.fl_dst = msg.range
        avg_dst = (self.fr_dst + self.fl_dst) / 2

        if avg_dst < DESIRED_DIST:
            self.cmd.linear.x = LINEAR_GAIN*(avg_dst - DESIRED_DIST)
        else:
            self.cmd.linear.x = 0
        
        self.cmd_publisher_.publish(self.cmd)

    def angular_move_callback(self, msg):
        self.get_logger().info(f"follow recieving angular offset {msg}")
        
        ang_vel = ANGULAR_GAIN*(CAMERA_WIDTH/2 - msg.data)
        if ang_vel <= -MIN_ANG_VEL or ang_vel >= MIN_ANG_VEL:
            self.cmd.angular.z = max(-MAX_ANG_VEL, min(ang_vel, MAX_ANG_VEL))
        self.get_logger().info(f"publishing cmd_vel: {self.cmd}")
        self.cmd_publisher_.publish(self.cmd)
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    follow = Follow()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(follow)
    # Explicity destroy the node
    follow.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()