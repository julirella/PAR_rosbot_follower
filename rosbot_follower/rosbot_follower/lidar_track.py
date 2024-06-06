import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Float64
from std_msgs.msg import Bool

from math import sin, radians

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
        self.angular_offset_pub = self.create_publisher(Float64, '/follow/lidar_angle', 10)
        self.runs = 20
        self.do_compute = True
        self.to_print = True
    
        self.get_logger().info("***************lidar track launched********************")

    # To get a reading run file and then in another terminal publish below
    # ros2 topic pub -1 /reading std_msgs/msg/Bool "data: true"

    def compute(self, data):
        if data:
            self.do_compute = True

    def laser_callback(self,msg):
        #self.runs -= 1

        if self.do_compute:
            self.find_edges(msg.ranges)
            self.get_logger().info(f"Scan processed")
            
        #if self.runs <= 0:
        #    self.do_compute = False

    def find_edges(self, ranges):
        first_edge = None
        last_edge = None
        count = 0
        range_thresh = 0.02
        count_thresh = 3

        for index, range in enumerate(ranges):
            # find an edge
            if range != float('inf') and not first_edge:
                first_edge = (index, range)
                last_edge = (index, range)
                #print(f"{index/2}: {range} - FIRST")   
            # if difference between edges is small enough
            elif first_edge and self.within_threshold(last_edge[1], range):
                last_edge = (index, range)
                count = 0
                #print(f"{index/2}: {range} - ADD")
            # If non candidate add to count
            elif first_edge and count < count_thresh:
                count += 1
                #print(f"{index/2}: {range} - COUNT: {count}") 
            # if too many consecutive non candidates 
            elif count >= count_thresh: 
                #print("RESET")
                self.check_object(first_edge, last_edge, ranges)
                first_edge = None
                count = 0
            #else:
                #print(f"{index/2}: {range} - SKIP")
    
    def check_object(self, first_edge, last_edge, ranges):
        found = False
        basic_checks = False
        object_width = abs(first_edge[0] - last_edge[0])
        sensible_edges = 6 <= object_width <= 60

        middle_index = round(max(first_edge[0], last_edge[0]) - abs(first_edge[0] - last_edge[0])/2)
        
        if ranges[middle_index] != float('inf'):
            basic_checks = self.basic_checks(first_edge, last_edge, middle_index, ranges)
        elif ranges[middle_index + 1] != float('inf'):
            basic_checks = self.basic_checks(first_edge, last_edge, middle_index + 1, ranges)
        elif ranges[middle_index - 1] != float('inf'):
            basic_checks = self.basic_checks(first_edge, last_edge, middle_index - 1, ranges)

        #print(f"Sensible_edges: {object_width}")

        if sensible_edges and basic_checks:
            found = True
            #print(f"CHECK THIS OBJECT: ({first_edge[0]/2},{first_edge[1]}, ({last_edge[0]/2},{last_edge[1]})")
            self.calculate_radius(first_edge, last_edge, middle_index)
        
        return found

    def basic_checks(self, first_edge, last_edge, middle_index, ranges):
        
        #print(f"Given: ({first_edge[0]/2},{first_edge[1]}) ({last_edge[0]/2},{last_edge[1]})")
        #print(f"Middle index: {middle_index/2}, {ranges[middle_index]}")

        max_edge = max(first_edge[1], last_edge[1])
        min_edge = min(first_edge[1], last_edge[1])
        # allow for greater error margin as distance increases
        threshold_factor = min(0.011, 0.007*max_edge)

        max_lower_thresh = 0.028 - threshold_factor
        min_lower_thresh = 0.017 - threshold_factor
        upper_thresh = 0.06
        difference_thresh = 0.025

        compare_edge_middle = max_lower_thresh < max_edge - ranges[middle_index] < upper_thresh
        compare_edge2_middle = min_lower_thresh < min_edge - ranges[middle_index] < upper_thresh
        compare_edge_edge = abs(first_edge[1] - last_edge[1]) < difference_thresh

        possibly_circular = compare_edge_middle and compare_edge2_middle and compare_edge_edge
        
        #print(f"Possible: {possibly_circular}")
        #print(f"cem: {compare_edge_middle} {max_lower_thresh} < {max_edge - ranges[middle_index]} < {upper_thresh}")
        #print(f"ce2m: {compare_edge2_middle} {min_lower_thresh} < {min_edge - ranges[middle_index]} < {upper_thresh}")
        #print(f"cee: {compare_edge_edge} {abs(first_edge[1] - last_edge[1])} < {difference_thresh}")        
        return possibly_circular
    
    def calculate_radius(self, first_edge, last_edge, middle_index):
        angle = radians(abs(first_edge[0]/2 - last_edge[0]/2))
        hypotenuse = max(first_edge[1], last_edge[1])
        radius = sin(angle/2)*hypotenuse
        #print(f"angle: {abs(first_edge[0]/2 - last_edge[0]/2)} is {radians} radians")
        #print(f"hypotenuse: {hypotenuse}")
        #print(f"RADIUS: {radius}")
        if 0.07 < radius < 0.1:
            #print(f"CANDIDATE")
            self.publish_angle(middle_index)

    def within_threshold(self, last_range, range):
        within = False
        threshold = last_range/35

        if abs(last_range - range) < threshold:
            within = True
        
        return within

    def publish_angle(self, index):
        msg = Float64()
        msg.data = float(index/2)
        
        self.get_logger().info(f"Angle sent: {msg}")
        self.angular_offset_pub.publish(msg)

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