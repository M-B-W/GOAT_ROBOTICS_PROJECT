#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time

class ButlerTask1(Node):
    def __init__(self):
        super().__init__('butler_task1')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # Define waypoints
        self.goals = {
            "home": [0.0, 0.0, 0.0],
            "kitchen": [-1.60, -1.71, 0.0],
            "table1": [-2.93, 1.69, 0.0]
        }

        self.run_task()

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0  # Facing forward
        return pose

    def run_task(self):
        for point in ['kitchen', 'table1', 'home']:
            coords = self.goals[point]
            goal_pose = self.create_pose(*coords)
            self.navigator.goToPose(goal_pose)

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info(f'Heading to {point}...')

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Reached {point}')
            else:
                self.get_logger().warn(f'Failed to reach {point}')

        rclpy.shutdown()

def main():
    rclpy.init()
    node = ButlerTask1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

