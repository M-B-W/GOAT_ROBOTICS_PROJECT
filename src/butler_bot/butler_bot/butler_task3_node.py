#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import sys

# if len(sys.argv) > 1:
#     selected_table = sys.argv[1]
# else:
#     selected_table = "table1"

# print(f"[INFO] Selected Table: {selected_table}")

# You can use `selected_table` in your logic below

class ButlerTask3(Node):
    def __init__(self):
        super().__init__('butler_task3_node')

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.confirm_stage = None  # 'kitchen' or 'table'
        self.confirmed = False

        # Waypoints: [x, y] for now
        self.goals = {
            "home": [0.0, 0.0],
            "kitchen": [-0.34, -1.6],
            "table1": [-2.93, 1.69],
            "table2": [-0.55, 1.52],
            "table3": [3.21, 1.92]
        }

        self.declare_parameter('timeout_seconds', 10)
        self.timeout = self.get_parameter('timeout_seconds').value

        # ✅ Declare and get the table name parameter
        self.declare_parameter('table', 'table1')
        self.table_target = self.get_parameter('table').get_parameter_value().string_value
        self.get_logger().info(f"Target table received as parameter: {self.table_target}")

        # Subscribe to confirmation topic
        self.create_subscription(String, '/confirm_stage', self.confirm_callback, 10)

        self.run_task()

    def confirm_callback(self, msg):
        if msg.data == self.confirm_stage:
            self.get_logger().info(f"Confirmation received at {self.confirm_stage}!")
            self.confirmed = True

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
            self.get_logger().error(f"Location '{location}' is not defined in waypoints.")
            return False

        x, y = self.goals[location]
        goal_pose = self.create_pose(x, y)
        self.navigator.goToPose(goal_pose)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f"Going to {location}...")
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Reached {location}")
            return True
        else:
            self.get_logger().warn(f"Failed to reach {location}")
            return False

    def wait_for_confirmation(self, stage):
        self.confirm_stage = stage
        self.confirmed = False
        self.get_logger().info(f"Waiting for confirmation at {stage} (timeout = {self.timeout}s)...")

        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while not self.confirmed:
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time - start_time > self.timeout:
                self.get_logger().warn(f"Timeout at {stage}!")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        return True

    def run_task(self):
        self.get_logger().info("Starting Test Case 3")

        # Validate table exists
        if self.table_target not in self.goals:
            self.get_logger().error(f"Table '{self.table_target}' is not defined.")
            self.go_to("home")
            rclpy.shutdown()
            return

        self.go_to("kitchen")
        confirmed_kitchen = self.wait_for_confirmation("kitchen")

        if not confirmed_kitchen:
            self.go_to("home")
            rclpy.shutdown()
            return

        self.go_to(self.table_target)
        confirmed_table = self.wait_for_confirmation("table")

        if not confirmed_table:
            self.get_logger().warn("No confirmation at table. Returning to kitchen before going home.")
            self.go_to("kitchen")  
        self.go_to("home")
        self.get_logger().info("Test Case 3 complete. Shutting down.")
        rclpy.shutdown()



def main():
    rclpy.init()
    node = ButlerTask3()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
