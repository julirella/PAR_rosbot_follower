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
ANGULAR_GAIN = 0.01
MIN_ANG_VEL = 0.15
MAX_ANG_VEL = 0.5
DESIRED_DIST = 0.6
MIN_LIN_VEL  = 0.05
MAX_LIN_VEL  = 0.4
LINEAR_GAIN  = 0.4
MIN_DIST = 0.3

class MotionController(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('motion_controller')

        # create the publisher object
        self.cmd_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # create subscribers       
        self.laser_subscriber = self.create_subscription(
            LaserScan, 
            'scan', 
            self.laser_callback, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        # Topic is published in Navigator
        self.angular_speed_sub = self.create_subscription(Float64, 'follow/angular_speed', self.angular_speed_callback, 10)
        self.move_sub = self.create_subscription(Bool, 'follow/motion_move', self.move_callback, 10)
       
        # self.fl_sub = self.create_subscription(Range, 'range/fl', self.fl_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # self.fr_sub = self.create_subscription(Range, 'range/fr', self.fr_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        # laser scan variables
        # self.fr_dst = 0
        # self.fl_dst = 0
        self.laser_forward = 0.0

        self.ang_vel = 0.0
        
        # flags
        self.go = False
        self.first = False

        # create a Twist message
        self.cmd = Twist()
        self.get_logger().info("***************motion controller launched********************")


    # Method to divide laser scan readings into sections
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
    
    # Set forward speed based on latest laser scan
    def laser_callback(self,msg):
        sections = self.divide_sections(msg)
        self.laser_forward = sections['front']
        self.linear_vel_controller()


        # front_dst = sections['front']

        # if front_dst < DESIRED_DIST:
            # self.cmd.linear.x = 0.0 
            # self.get_logger().info(f"something is too close")    
        # else:
        #     self.cmd.linear.x = LINEAR_GAIN*(front_dst - DESIRED_DIST)
            # self.get_logger().info(f"ready to go, linear velocity: {self.cmd.linear.x}")

    # Publishes to cmd_vel using latest angular and laser data (see laser_callback)
    def angular_speed_callback(self, msg):
        self.ang_vel = msg.data
        if self.first == False:
            self.first = True

        self.linear_vel_controller()
  
        # self.get_logger().info(f"publishing cmd_vel: {self.cmd}")
        # self.cmd_publisher_.publish(self.cmd)
    
    def move_callback(self, msg):
        self.get_logger().info("move recieved")
        self.go = msg.data
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.cmd_publisher_.publish(self.cmd)

    def linear_vel_controller(self):
        # self.get_logger().info(f"laser forward: {self.laser_forward}\n angular_vel: {self.ang_vel}")

        if self.go == True:
            self.cmd.angular.z = self.ang_vel
            
            THRESHOLD = 0.45
            pos_ang = abs(self.ang_vel)
            if self.first == True and self.laser_forward > DESIRED_DIST and pos_ang < 0.8:
                self.cmd.linear.x = LINEAR_GAIN*(self.laser_forward - DESIRED_DIST) #maybe

            # Reduce forward gain as the angle increases (above threshold)
                if pos_ang > THRESHOLD:
                    # 1.60 - 0.45 =1.15*2 = 2.3
                    scalar = (pos_ang - THRESHOLD)*2
                    linear_velocity = self.cmd.linear.x - (self.cmd.linear.x*scalar)
                    # self.get_logger().info(f"Above threshold, linear velocity: {linear_velocity}, reduced from {self.cmd.linear.x}")
                    if linear_velocity < 0.0: #we never want to reverse
                        self.cmd.linear.x = 0.0
                    else:
                        self.cmd.linear.x = linear_velocity
            else:
                self.cmd.linear.x = 0.0
            
            self.get_logger().info(f"publishing velocity lin: {self.cmd.linear.x}, ang: {self.cmd.angular.z}")
            self.cmd_publisher_.publish(self.cmd)

    # def fr_callback(self, msg):
    #     self.fr_dst = msg.range
    #     # self.get_logger().info(f"right: {self.fr_dst}")
    #     self.linear_vel_controller()
                
    # def fl_callback(self, msg):
    #     self.fl_dst = msg.range
    #     # self.get_logger().info(f"left: {self.fl_dst}")
    #     if msg.range == float('inf'):
    #        print("INF") 
    #     elif msg.range == float('-inf'):
    #         print("-INF")
    #     elif msg.range == float('nan'):
    #         print("Caught NAN")
    #     self.linear_vel_controller()

        # avg_dst = (self.fr_dst + self.fl_dst) / 2

        # if avg_dst < DESIRED_DIST:
        #     self.cmd.linear.x = LINEAR_GAIN*(avg_dst - DESIRED_DIST)
        # else:
        #     self.cmd.linear.x = 0
        
        # self.cmd_publisher_.publish(self.cmd)

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    motion_controller = MotionController()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(motion_controller)
    # Explicity destroy the node
    motion_controller.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()