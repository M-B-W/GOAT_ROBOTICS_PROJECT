#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class ButlerTask6(Node):
    def __init__(self):
        super().__init__('butler_task6_node')

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # Get multiple tables from CLI
        self.declare_parameter('tables', ['table1', 'table2', 'table3'])
        self.table_targets = self.get_parameter('tables').get_parameter_value().string_array_value

        # Confirmation flags
        self.confirm_stage = None
        self.confirmed = False

        self.goals = {
            "home": [0.0, 0.0],
            "kitchen": [-0.34, -1.6],
            "table1": [-2.93, 1.69],
            "table2": [-0.55, 1.52],
            "table3": [3.21, 1.92]
        }

        self.create_subscription(String, '/confirm_stage', self.confirm_callback, 10)

        self.timeout = 10  # seconds
        self.run_task()

    def confirm_callback(self, msg):
        if msg.data == self.confirm_stage:
            self.confirmed = True
            self.get_logger().info(f"✅ Confirmation received at {self.confirm_stage}")

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

    def wait_for_confirmation(self, stage):
        self.confirm_stage = stage
        self.confirmed = False
        self.get_logger().info(f"🕓 Waiting for confirmation at {stage} for {self.timeout}s...")

        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while not self.confirmed:
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time - start_time > self.timeout:
                self.get_logger().warn(f"❌ Timeout at {stage}. Skipping...")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def run_task(self):
        self.get_logger().info(f"📦 Task 6: Delivering to tables: {self.table_targets}")

        if not self.go_to("kitchen"):
            self.get_logger().error("❌ Couldn't reach kitchen. Aborting.")
            return

        for table in self.table_targets:
            self.get_logger().info(f"➡️ Attempting delivery to {table}")
            if self.go_to(table):
                confirmed = self.wait_for_confirmation("table")
                if not confirmed:
                    self.get_logger().warn(f"⏩ Skipping {table} (no confirmation)")
            else:
                self.get_logger().warn(f"⚠️ Could not reach {table}, skipping.")

        self.get_logger().info("🔄 Returning to kitchen before heading home.")
        self.go_to("kitchen")
        self.go_to("home")
        self.get_logger().info("✅ Task 6 complete.")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ButlerTask6()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
