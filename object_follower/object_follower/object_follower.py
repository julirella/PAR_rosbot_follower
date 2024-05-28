import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np

class objectFollower(Node):
    def __init__(self):
        super().__init__("Object_follower")
        self.set_parameters()
        self.create_subscribers()
        self.create_publishers()
        self.log_message("Object Follower Node Initialized")

    def set_parameters(self):
        self.threshold = 0.8  # object distance threshold
        self.curr_state = 0  #start with find a object
        self.object_found = False  #inital with object not find
        self.start_received = False  # wait to start
        self.found_received = False  # wait camera to confirm
    #create subscribers
    def create_subscribers(self):
        self.scan_subscription = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.start_subscription = self.create_subscription(
            Bool, "/start", self.start_callback, 10
        )
        self.found_subscription = self.create_subscription(
            Bool, "/found", self.found_callback, 10
        )
    #create publisher
    def create_publishers(self):
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def scan_callback(self, msg):
        sections = self.divide_sections(msg)
        self.make_move(sections)

    def start_callback(self, msg):
        if msg.data :
            self.log_message("Starting Object Following")
            self.start_received = True
        else:
            self.log_message("Stopping Object following")
            self.start_received = False

    def found_callback(self, msg):
        if msg.data:
            self.log_message("Found Object")
            self.found_received = True
        else:
            self.found_received = False

    #divide laser read into 5 sections
    def divide_sections(self, msg):
        length = len(msg.ranges)
        each_div = length // 8
        front_start = length - each_div // 2
        front_end = each_div // 2

        sections = {
            "front": min(min(msg.ranges[front_start:]), min(msg.ranges[:front_end])),
            "left": min(msg.ranges[each_div : 2 * each_div]),
            "fleft": min(msg.ranges[each_div // 2 : each_div]),
            "fright": min(msg.ranges[front_start - each_div // 2 : front_start]),
            "right": min(msg.ranges[length - 2 * each_div : length - each_div]),
            "rright": min(msg.ranges[length - each_div : length - each_div // 2]),
            "rleft": min(msg.ranges[2 * each_div : 2 * each_div + each_div // 2]),
        }
        return sections

    def make_move(self, sections):
        min_distance = 0.5  # Minimum distance to maintain from the target

        if not self.object_found:
            if (
                sections["front"] > self.threshold
                and sections["right"] > self.threshold
                and sections["left"] > self.threshold
                and sections["rright"] > self.threshold
                and sections["rleft"] > self.threshold
            ):
                self.curr_state = 0  # Object not found
            elif self.threshold < sections["front"] and self.found_received == True:
                self.curr_state = 1  # Found object, move towards target
                self.log_message("Moving towards target")
        else:
            if sections["front"] > min_distance:
                self.curr_state = 2  # Move forward
                self.log_message("Move forward")
            elif sections["fleft"] < sections["fright"]:
                self.curr_state = 3  # Adjust left
                self.log_message("Adjust left")
            elif sections["fleft"] > sections["fright"]:
                self.curr_state = 4  # Adjust right
                self.log_message("Adjust right")
            else:
                self.curr_state = 5  # Maintain distance
                self.log_message("Maintain distance")

    def run(self):
        self.log_message("Object Follower Started")
        while rclpy.ok():
            if self.start_received:
                vel_cmd = self.get_velocity_command()
                self.cmd_vel_publisher.publish(vel_cmd)
            else:
                self.log_message("Waiting for start message...")

            rclpy.spin_once(self, timeout_sec=0.1)

    def get_velocity_command(self):
        vel_cmd = Twist()
        if self.curr_state == 0:
            self.log_message("Object not found")
        elif self.curr_state == 1:
            vel_cmd.linear.x = 0.15
            vel_cmd.angular.z = 0.0
            self.log_message("Moving towards target")
        elif self.curr_state == 2:
            vel_cmd.linear.x = 0.15
            vel_cmd.angular.z = 0.0
            self.log_message("Move forward")
        elif self.curr_state == 3:
            vel_cmd.linear.x = 0.1
            vel_cmd.angular.z = 0.2
            self.log_message("Adjust left")
        elif self.curr_state == 4:
            vel_cmd.linear.x = 0.1
            vel_cmd.angular.z = -0.2
            self.log_message("Adjust right")
        elif self.curr_state == 5:
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 0.0
            self.log_message("Maintain distance")
        else:
            self.log_message("Unknown state", "warn")
        return vel_cmd

    def log_message(self, message, level="info"):
        if level == "info":
            self.get_logger().info(message)
        elif level == "warn":
            self.get_logger().warn(message)
        elif level == "error":
            self.get_logger().error(message)

def main(args=None):
    rclpy.init(args=args)
    object_follower = objectFollower()
    object_follower.run()
    object_follower.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()