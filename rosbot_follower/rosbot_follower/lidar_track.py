import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
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
        range_thresh = 0.02
        count_thresh = 3

        for index, range in enumerate(ranges):
            # find an edge
            if range != float('inf') and not first_edge:
                first_edge = (index, range)
                last_edge = (index, range)
                print(f"{index/2}: {range} - FIRST")   
            # if difference between edges is small enough
            elif first_edge and self.within_threshold(last_edge[1], range):
                last_edge = (index, range)
                count = 0
                print(f"{index/2}: {range} - ADD")
            # If non candidate add to count
            elif first_edge and count < count_thresh:
                count += 1
                print(f"{index/2}: {range} - COUNT: {count}") 
            # if too many consecutive non candidates 
            elif count >= count_thresh: 
                print("RESET")
                self.check_object(first_edge, last_edge, ranges)
                first_edge = None
                count = 0
            else:
                print(f"{index/2}: {range} - SKIP")
            #print(f"{index/2} is {range}")
    
    def check_object(self, first_edge, last_edge, ranges):
        found = False
        object_width = abs(first_edge[0] - last_edge[0])
        sensible_edges = 3 <= object_width <= 60

        # allow for greater error margin as distance increases
        difference_threshold = 0.02 + 0.015*first_edge[1]
        
        middle_index = round(max(first_edge[0], last_edge[0]) - abs(first_edge[0] - last_edge[0])/2)

        #print(f"Given: {first_edge}, {last_edge}")
        #print(f"Middle index: {middle_index}")
        if ranges[middle_index] != float('inf'):
            edge = max(first_edge[1], last_edge[1])
            possibly_circular = 0.01 < edge - ranges[middle_index] < 0.13
        #    print(f"Possible_1: {edge - ranges[middle_index]}") 
        else:
            possibly_circular = abs(first_edge[1] - last_edge[1]) < difference_threshold
        #    print(f"Possible_2: {abs(first_edge[1] - last_edge[1])} < {difference_threshold}")
        #print(f"Sensible_edges: {object_width}")

        if sensible_edges and possibly_circular:
            found = True
            print(f"CHECK THIS OBJECT: ({first_edge[0]/2},{first_edge[1]}, ({last_edge[0]/2},{last_edge[1]})")
            self.calculate_radius(first_edge, last_edge)
        
        return found

    def calculate_radius(self, first_edge, last_edge):
        angle = radians(abs(first_edge[0]/2 - last_edge[0]/2))
        hypotenuse = max(first_edge[1], last_edge[1])
        radius = sin(angle/2)*hypotenuse
        print(f"angle: {abs(first_edge[0]/2 - last_edge[0]/2)} is {radians} radians")
        print(f"hypotenuse: {hypotenuse}")
        print(f"RADIUS: {radius}")
        if 0.04 < radius < 0.1:
            print(f"CANDIDATE")

    def within_threshold(self, last_range, range):
        within = False
        threshold = last_range/35

        if abs(last_range - range) < threshold:
            within = True
        
        return within

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