#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class ButlerTask7(Node):
    def __init__(self):
        super().__init__('butler_task7_node')

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # Get table list from parameter
        self.declare_parameter('tables', ['table1', 'table2', 'table3'])
        self.table_targets = list(self.get_parameter('tables').get_parameter_value().string_array_value)

        # Store canceled table names
        self.canceled_tables = set()

        self.goals = {
            "home": [0.0, 0.0],
            "kitchen": [-0.34, -1.6],
            "table1": [-2.93, 1.69],
            "table2": [-0.55, 1.52],
            "table3": [3.21, 1.92]
        }

        self.create_subscription(String, '/cancel_task', self.cancel_callback, 10)

        self.run_task()

    def cancel_callback(self, msg):
        self.get_logger().warn(f"❌ Cancel received: {msg.data}")
        self.canceled_tables.add(msg.data.strip())

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
            self.get_logger().error(f"'{location}' is not in the goal list!")
            return False

        x, y = self.goals[location]
        pose = self.create_pose(x, y)
        self.navigator.goToPose(pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f"Navigating to {location}...")

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"✅ Reached {location}")
            return True
        else:
            self.get_logger().warn(f"⚠️ Failed to reach {location}")
            return False

    def run_task(self):
        self.get_logger().info(f"🚀 Starting Task 7 for tables: {self.table_targets}")

        if not self.go_to("kitchen"):
            self.get_logger().error("❌ Could not reach kitchen. Aborting.")
            return

        for table in self.table_targets:
            if table in self.canceled_tables:
                self.get_logger().warn(f"⏩ Skipping {table} (order canceled)")
                continue

            if self.go_to(table):
                self.get_logger().info(f"📦 Delivered to {table}")
            else:
                self.get_logger().warn(f"⚠️ Could not reach {table}")

        self.get_logger().info("🔄 Returning to kitchen before home")
        self.go_to("kitchen")
        self.go_to("home")
        self.get_logger().info("✅ Task 7 complete. Shutting down.")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ButlerTask7()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
