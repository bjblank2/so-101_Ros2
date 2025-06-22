#!/usr/bin/env python3
"""
Test script for leader-follower arm teleoperation
This script demonstrates how to use the leader and follower arm nodes
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math


class LeaderFollowerTest(Node):
    def __init__(self):
        super().__init__('leader_follower_test')
        
        # Subscribe to both leader and follower joint states
        self.leader_subscription = self.create_subscription(
            JointState,
            'leader_joint_states',
            self.leader_callback,
            10
        )
        
        self.follower_subscription = self.create_subscription(
            JointState,
            'follower_joint_states',
            self.follower_callback,
            10
        )
        
        self.leader_joint_states = None
        self.follower_joint_states = None
        
        # Timer to print status
        self.timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info("Leader-Follower test node started")
        self.get_logger().info("Listening to leader_joint_states and follower_joint_states topics")
    
    def leader_callback(self, msg):
        self.leader_joint_states = msg
        self.get_logger().info(f"Received leader joint states: {[f'{name}: {pos:.2f}' for name, pos in zip(msg.name, msg.position)]}")
    
    def follower_callback(self, msg):
        self.follower_joint_states = msg
        self.get_logger().info(f"Received follower joint states: {[f'{name}: {pos:.2f}' for name, pos in zip(msg.name, msg.position)]}")
    
    def print_status(self):
        if self.leader_joint_states is None:
            self.get_logger().warn("No leader joint states received yet")
        else:
            self.get_logger().info("Leader arm is publishing joint states")
            
        if self.follower_joint_states is None:
            self.get_logger().warn("No follower joint states received yet")
        else:
            self.get_logger().info("Follower arm is publishing joint states")


def main(args=None):
    rclpy.init(args=args)
    test_node = LeaderFollowerTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 