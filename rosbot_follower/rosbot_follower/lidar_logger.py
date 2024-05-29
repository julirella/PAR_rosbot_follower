import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Bool
import numpy as np

import geometry_msgs.msg
import tf2_geometry_msgs
import tf2_ros

class LidarLogger(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('lidar_logger')
        # create the publisher object
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        #self.laser_forward = 0

        #self.timer = self.create_timer(1.0, self.transform)
    # def divide_sections(self, msg):
    #     length = len(msg.ranges)
    #     each_div = length // 8
    #     front_start = length - each_div // 2
    #     front_end = each_div // 2

    #     sections = {
    #         "front": min(min(msg.ranges[front_start:]), min(msg.ranges[:front_end])),
    #         "fleft": min(msg.ranges[each_div // 2 : each_div]),
    #         "fright": min(msg.ranges[front_start - each_div // 2 : front_start]),
    #     }
    #     return sections
    
    def laser_callback(self,msg):
        # Save the frontal laser scan info at 0°
        if msg:
            for reading in range(-45, 45):
               self.get_logger().info(f'{reading}: {msg.ranges[reading]}')

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    lidar_logger = LidarLogger()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(lidar_logger)
    # Explicity destroy the node
    lidar_logger.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()