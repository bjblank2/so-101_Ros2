#!/usr/bin/env python3
"""
Calibration client for leader-follower arm teleoperation
This script demonstrates how to use the calibration services
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time


class CalibrationClient(Node):
    def __init__(self):
        super().__init__('calibration_client')
        
        # Create service clients
        self.calibrate_client = self.create_client(Trigger, 'calibrate_arms')
        self.home_client = self.create_client(Trigger, 'home_arms')
        self.set_home_client = self.create_client(Trigger, 'set_home_position')
        self.match_client = self.create_client(Trigger, 'match_positions')
        
        self.get_logger().info("Calibration client started")
        self.get_logger().info("Available commands:")
        self.get_logger().info("  calibrate() - Run full calibration routine")
        self.get_logger().info("  home_arms() - Move both arms to home position")
        self.get_logger().info("  set_home() - Set current position as home")
        self.get_logger().info("  match_positions() - Match follower to leader")
    
    def wait_for_service(self, client, service_name):
        """Wait for a service to be available"""
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {service_name} service...')
    
    def call_service(self, client, service_name):
        """Call a service and return the response"""
        self.wait_for_service(client, service_name)
        
        request = Trigger.Request()
        future = client.call_async(request)
        
        self.get_logger().info(f'Calling {service_name}...')
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                try:
                    response = future.result()
                    if response.success:
                        self.get_logger().info(f'{service_name} succeeded: {response.message}')
                    else:
                        self.get_logger().error(f'{service_name} failed: {response.message}')
                    return response
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {e}')
                    return None
    
    def calibrate(self):
        """Run full calibration routine"""
        return self.call_service(self.calibrate_client, 'calibrate_arms')
    
    def home_arms(self):
        """Move both arms to home position"""
        return self.call_service(self.home_client, 'home_arms')
    
    def set_home(self):
        """Set current position as home position"""
        return self.call_service(self.set_home_client, 'set_home_position')
    
    def match_positions(self):
        """Match follower position to leader position"""
        return self.call_service(self.match_client, 'match_positions')


def main(args=None):
    rclpy.init(args=args)
    client = CalibrationClient()
    
    try:
        # Example usage - uncomment the commands you want to run
        
        # Step 1: Move arms to home position
        # client.get_logger().info("Step 1: Moving arms to home position...")
        # client.home_arms()
        # time.sleep(2.0)
        
        # Step 2: Set current position as home (optional)
        # client.get_logger().info("Step 2: Setting current position as home...")
        # client.set_home()
        # time.sleep(1.0)
        
        # Step 3: Run full calibration
        # client.get_logger().info("Step 3: Running full calibration...")
        # client.calibrate()
        # time.sleep(2.0)
        
        # Step 4: Test position matching
        # client.get_logger().info("Step 4: Testing position matching...")
        # client.match_positions()
        
        # Keep the node running for manual testing
        client.get_logger().info("Calibration client ready. Use the methods to test calibration.")
        client.get_logger().info("Example: client.calibrate()")
        
        rclpy.spin(client)
        
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 