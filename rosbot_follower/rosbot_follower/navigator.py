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
MAX_ANG_VEL = 1.8
DESIRED_DIST = 0.6
MIN_LIN_VEL  = 0.05
MAX_LIN_VEL  = 0.4
LINEAR_GAIN  = 0.4

class Navigator(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('navigator')

        # create the publisher object, motion_controller is the sole subscriber
        self.angular_speed_publisher = self.create_publisher(Float64, 'follow/angular_speed', 10)

        # create subscriber, main controller is the sole publisher       
        self.anglular_offset_subscriber = self.create_subscription(Float64, '/follow/main_angle', self.angular_move_callback, 10)
    
        self.get_logger().info("***************navigator launched********************")

    def angular_move_callback(self, msg):
        # self.get_logger().info(f"follow recieving angle {msg}")
        ang_vel = ANGULAR_GAIN*msg.data #TODO: IF IT BRAKES, REMOVE MINUS!!!!!
        
        if ang_vel <= -MIN_ANG_VEL or ang_vel >= MIN_ANG_VEL:
            ang_vel = max(-MAX_ANG_VEL, min(ang_vel, MAX_ANG_VEL))

        msg = Float64()
        msg.data = ang_vel
        # self.get_logger().info(f"publishing angular speed: {msg}")
        self.angular_speed_publisher.publish(msg)
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    navigator = Navigator()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(navigator)
    # Explicity destroy the node
    navigator.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()