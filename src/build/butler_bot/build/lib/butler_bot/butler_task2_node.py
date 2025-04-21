#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import time

class ButlerTask2(Node):
    def __init__(self):
        super().__init__('butler_task2_node')

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.confirmed = False
        self.timeout_duration = 10  # seconds

        self.goals = {
            "home": [0.0, 0.0],
            "kitchen": [2.5, 1.0],
            "table1": [4.0, 2.0]
        }

        # Subscribe to /confirm topic
        self.create_subscription(Bool, '/confirm', self.confirm_callback, 10)

        # Start the task
        self.run_task()

    def confirm_callback(self, msg):
        if msg.data:
            self.confirmed = True
            self.get_logger().info("Confirmation received!")

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0  # face forward
        return pose

    def go_to(self, location_name):
        x, y = self.goals[location_name]
        goal_pose = self.create_pose(x, y)
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'Going to {location_name}...')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Reached {location_name}")
        else:
            self.get_logger().warn(f"Failed to reach {location_name}")

    def run_task(self):
        self.get_logger().info("Starting Test Case 2")

        self.go_to("kitchen")

        self.get_logger().info("Waiting for confirmation for up to 10 seconds...")

        # Wait for confirmation or timeout
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while not self.confirmed:
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time - start_time >= self.timeout_duration:
                self.get_logger().warn("Timeout! No confirmation received.")
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.confirmed:
            self.go_to("table1")

        self.go_to("home")
        self.get_logger().info("Task Case 2 complete. Shutting down.")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = ButlerTask2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


