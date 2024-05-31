import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Bool

class LidarLogger(Node):

    def __init__(self):
        super().__init__('lidar_logger')
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.follow_subscriber = self.create_subscription(
                Bool,
                '/reading',
                self.reading,
                10
        )
        self.scan = None
    
    # To get a reading run file and then in another terminal publish below
    # ros2 topic pub -1 /reading std_msgs/msg/Bool "data: true"

    def reading(self, data):
        msg = self.scan
        if data and msg:
            for reading in range(-90, 90):
               self.get_logger().info(f'{reading/2}: {msg.ranges[reading]}')

    def laser_callback(self,msg):
        if msg:
            self.scan = msg

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