#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess

class MissionSwitcher(Node):
    def __init__(self):
        super().__init__('mission_switcher')
        self.subscription = self.create_subscription(
            Bool,
            '/switch_to_navigation',
            self.switch_callback,
            10
        )
        self.exploration_running = True
        self.get_logger().info("Waiting for /switch_to_navigation...")

    def switch_callback(self, msg):
        if msg.data and self.exploration_running:
            self.get_logger().info("Switch triggered! Stopping exploration and starting navigation.")

            # Kill explorer (if launched from separate launch)
            subprocess.run(['pkill', '-f', 'explore_robotx.launch'])

            # Start navigation
            subprocess.Popen(['ros2', 'launch', 'smb_bringup', 'smb_sim_navigation.launch.py'])

            # Optionally send a goal
            goal = """header:
                    stamp:
                        sec: 0
                        nanosec: 0
                    frame_id: ''
                    point:
                    x: -3.0
                    y: 2.0
                    z: 0.0
                    """
            subprocess.run([
                'ros2', 'topic', 'pub', '/goal_point', 'geometry_msgs/msg/PointStamped', goal, '--once'
            ])

            self.exploration_running = False

