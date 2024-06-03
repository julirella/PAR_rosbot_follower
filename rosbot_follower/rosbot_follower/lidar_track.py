import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Bool

class LidarTrack(Node):

    def __init__(self):
        super().__init__('lidar_track')
        self.laser_subscriber = self.create_subscription(
                LaserScan, 
                '/scan', 
                self.laser_callback, 
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.follow_subscriber = self.create_subscription(
                Bool,
                '/compute_lidar',
                self.compute,
                10
        )
        self.do_compute = True
    
    # To get a reading run file and then in another terminal publish below
    # ros2 topic pub -1 /reading std_msgs/msg/Bool "data: true"

    def compute(self, data):
        if data:
            self.do_compute = True

    def laser_callback(self,msg):
        if self.do_compute:
            self.find_edges(msg.ranges)
            self.do_compute = False


    def find_edges(self, ranges):
        first_edge = None
        last_edge = None
        count = 0
        range_thresh = 0.2
        count_thresh = 4

        for index, range in enumerate(ranges):
            if first_edge and abs(range - first_edge[1]) < range_thresh:
                last_edge = (index, range)
                print(f"{index/2} ADD")
            elif range != float('inf') and not first_edge:
                first_edge = (index, range)
                print(f"{index/2} FIRST")    
            elif count > count_thresh:
                first_edge = None
                count = 0
            else:
                count += 1
            #print(f"{index/2} is {range}")
        
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    lidar_track = LidarTrack()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(lidar_track)
    # Explicity destroy the node
    lidar_track.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()