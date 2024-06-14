import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Bool

class LidarRepeater(Node):

    def __init__(self):
        super().__init__('lidar_repeater')

        self.laser_publisher = self.create_publisher(
                LaserScan, 
                'follow/scan', 
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.laser_subscriber = self.create_subscription(
                LaserScan, 
                '/scan', 
                self.laser_callback, 
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.follow_subscriber = self.create_subscription(
                Bool,
                '/reading',
                self.reading,
                10
        )
        self.scan = None

        self.get_logger().info("***************lidar repeater launched********************")
    
    # To get a reading run file and then in another terminal publish below
    # ros2 topic pub -1 /reading std_msgs/msg/Bool "data: true"

    def reading(self, data):
        msg = self.scan
        start = 0
        stop = 359
        if data and msg:
            for reading in range(start*2, stop*2):
               self.get_logger().info(f'{reading/2}: {msg.ranges[reading]}')

    def laser_callback(self,msg):
        self.scan = msg
        # Publish to repeater topic
        self.laser_publisher.publish(msg)

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    lidar_repeater = LidarRepeater()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(lidar_repeater)
    # Explicity destroy the node
    lidar_repeater.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()