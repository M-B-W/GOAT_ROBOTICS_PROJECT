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

class ButlerTask4(Node):
    def __init__(self):
        super().__init__('butler_task4_node')

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.declare_parameter('table', 'table1')
        self.table_target = self.get_parameter('table').get_parameter_value().string_value

        # Updated: Cancellation state per stage
        self.task_canceled = {'kitchen': False, 'table': False}

        self.goals = {
            "home": [0.0, 0.0],
            "kitchen": [-0.34, -1.6],
            "table1": [-2.93, 1.69],
            "table2": [-0.55, 1.52],
            "table3": [3.21, 1.50]
        }

        self.create_subscription(String, '/cancel_task', self.cancel_callback, 10)
        self.run_task()

    def cancel_callback(self, msg):
        stage = msg.data.lower()
        if stage in self.task_canceled:
            self.task_canceled[stage] = True
            self.get_logger().warn(f"Cancellation received for stage: {stage}")
        else:
            self.get_logger().warn(f"Invalid cancel stage: {msg.data}")

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def go_to(self, location, cancel_stage=None):
        if location not in self.goals:
            self.get_logger().error(f"{location} not in goals.")
            return False

        x, y = self.goals[location]
        pose = self.create_pose(x, y)
        self.navigator.goToPose(pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

            if cancel_stage and self.task_canceled[cancel_stage]:
                self.get_logger().warn(f"Canceled during navigation to {location} at stage: {cancel_stage}")
                self.navigator.cancelTask()
                return 'canceled'

            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f"Navigating to {location}...")

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Reached {location}")
            return 'reached'
        else:
            self.get_logger().warn(f"Failed to reach {location}")
            return 'failed'

    def run_task(self):
        self.get_logger().info(f"Starting Task Case 4 to {self.table_target}")

        result_kitchen = self.go_to("kitchen", cancel_stage='kitchen')
        if result_kitchen == 'canceled':
            self.get_logger().info("Canceled on the way to kitchen. Returning to home.")
            self.go_to("home")
            rclpy.shutdown()
            return

        result_table = self.go_to(self.table_target, cancel_stage='table')
        if result_table == 'canceled':
            self.get_logger().info("Canceled on the way to table. Returning to kitchen then home.")
            self.go_to("kitchen")
            self.go_to("home")
            rclpy.shutdown()
            return

        self.get_logger().info("Task finished successfully. Returning home.")
        self.go_to("home")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ButlerTask4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
