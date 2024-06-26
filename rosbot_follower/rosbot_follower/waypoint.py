#!/usr/bin/env python  

## This example is based on TheConstruct Nav2 Tutorial

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# Waypoint navigation is currently unused in current form
# Node will be re-purposed from navigating to recognised object in camera
# to navigating to predicated location when target is not detected
class NavToPoseActionClient(Node):

    def __init__(self):
        super().__init__('waypoint',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info('Starting')
        
        self.sub_be = self.create_subscription(
                            PoseStamped,
                            '/follow/pose',
                            self.goal_listener,
                            10
                    )

    def goal_listener(self, sentPose):
        if sentPose:
            self.get_logger().info(f"Start goal")
            self.send_goal(sentPose)

    def send_goal(self, sentPose):
        self.get_logger().info('Sending Goal')
        
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.pose.position.x = sentPose.pose.position.x
        goal_pose.pose.pose.position.y = sentPose.pose.position.y
        goal_pose.pose.pose.position.z = sentPose.pose.position.z
        
        goal_pose.pose.pose.orientation.x = 1.0
        goal_pose.pose.pose.orientation.y = 0.0
        goal_pose.pose.pose.orientation.z = 0.0
        goal_pose.pose.pose.orientation.w = 0.0

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
            
        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
        else:
            self.get_logger().info('Navigation failed with status: {0}'.format(status))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

def main(args=None):
    rclpy.init(args=args)

    action_client = NavToPoseActionClient()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()