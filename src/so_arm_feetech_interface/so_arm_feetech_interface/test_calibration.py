#!/usr/bin/env python3
"""
Test script for the improved calibration procedure
This script demonstrates the complete calibration workflow
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time


class CalibrationTester(Node):
    def __init__(self):
        super().__init__('calibration_tester')
        
        # Create service clients
        self.calibrate_client = self.create_client(Trigger, 'calibrate_arms')
        self.home_client = self.create_client(Trigger, 'home_arms')
        self.match_client = self.create_client(Trigger, 'match_positions')
        self.apply_offsets_client = self.create_client(Trigger, 'apply_offsets')
        self.clear_offsets_client = self.create_client(Trigger, 'clear_offsets')
        
        self.get_logger().info("Calibration tester started")
    
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
    
    def run_calibration_test(self):
        """Run the complete calibration test procedure"""
        self.get_logger().info("=== Starting Calibration Test Procedure ===")
        
        # Step 1: Clear any existing offsets
        self.get_logger().info("\nStep 1: Clearing any existing calibration offsets...")
        self.call_service(self.clear_offsets_client, 'clear_offsets')
        time.sleep(2.0)
        
        # Step 2: Move arms to home position
        self.get_logger().info("\nStep 2: Moving both arms to home position...")
        self.call_service(self.home_client, 'home_arms')
        time.sleep(5.0)  # Wait for movement to complete
        
        # Step 3: Run calibration (read positions and save offsets)
        self.get_logger().info("\nStep 3: Running calibration - reading positions and calculating offsets...")
        self.call_service(self.calibrate_client, 'calibrate_arms')
        time.sleep(3.0)
        
        # Step 4: Test position matching with saved offsets
        self.get_logger().info("\nStep 4: Testing position matching using saved offsets...")
        self.call_service(self.match_client, 'match_positions')
        time.sleep(3.0)
        
        # Step 5: Apply offsets again to verify they work
        self.get_logger().info("\nStep 5: Applying saved offsets again to verify...")
        self.call_service(self.apply_offsets_client, 'apply_offsets')
        time.sleep(3.0)
        
        self.get_logger().info("\n=== Calibration Test Procedure Completed ===")
        self.get_logger().info("The calibration offsets have been saved and can be used automatically in future sessions.")


def main(args=None):
    rclpy.init(args=args)
    tester = CalibrationTester()
    
    try:
        # Run the calibration test
        tester.run_calibration_test()
        
        # Keep the node running for a bit to see the results
        tester.get_logger().info("Test completed. Keeping node alive for 10 seconds...")
        time.sleep(10.0)
        
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 