import rclpy
from rclpy import duration, time
from rclpy.node import Node

import rclpy.time
from std_msgs.msg import Float64



class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        #subscribers
        self.camera_sub = self.create_subscription(Float64, 'follow/camera_angle', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(Float64, 'follow/lidar_angle', self.lidar_callback, 10)

        #publishers
        self.angle_pub = self.create_publisher(Float64, 'follow/main_angle', 10)

        self.lastCamReading = time.Time()

        self.get_logger().info("***************main controller launched********************")


    def camera_callback(self, msg):
        self.get_logger().info(f"main controller receiving camera angle and sending it on")
        self.angle_pub.publish(msg)
        self.lastCamReading = self.get_clock().now()

    def lidar_callback(self, msg):
        timeSinceCam = self.get_clock().now() - self.lastCamReading
        if timeSinceCam > duration.Duration(seconds=0.5):
            self.get_logger().info(f"camera angle timed out, time since last msg: {timeSinceCam}")
            # self.angle_pub.publish(msg)        
    

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    main_controller = MainController()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(main_controller)
    # Explicity destroy the node
    main_controller.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    