#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class ButlerTask5(Node):
    def __init__(self):
        super().__init__('butler_task5_node')

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # Get multiple tables from CLI parameters
        self.declare_parameter('tables', ['table1', 'table2', 'table3'])
        self.table_targets = self.get_parameter('tables').get_parameter_value().string_array_value

        self.goals = {
            "home": [0.0, 0.0],
            "kitchen": [-0.34, -1.6],
            "table1": [-2.93, 1.69],
            "table2": [-0.55, 1.52],
            "table3": [3.21, 1.92]
        }

        self.run_task()

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def go_to(self, location):
        if location not in self.goals:
            self.get_logger().error(f"Location '{location}' not in goals!")
            return False

        x, y = self.goals[location]
        goal_pose = self.create_pose(x, y)
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f"🚶 Navigating to {location}...")

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"✅ Reached {location}")
            return True
        else:
            self.get_logger().warn(f"⚠️ Failed to reach {location}")
            return False

    def run_task(self):
        self.get_logger().info(f"📦 Task 5: Delivering to tables: {self.table_targets}")

        # Step 1: Go to Kitchen
        if not self.go_to("kitchen"):
            self.get_logger().error("❌ Couldn't reach kitchen. Aborting.")
            return

        # Step 2: Deliver to each table (no confirmation)
        for table in self.table_targets:
            self.get_logger().info(f"➡️ Delivering to {table}")
            if not self.go_to(table):
                self.get_logger().warn(f"⚠️ Could not reach {table}, skipping.")

        # Step 3: Return to Home
        self.get_logger().info("🏠 Returning home...")
        self.go_to("home")
        self.get_logger().info("✅ Task 5 complete.")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ButlerTask5()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
