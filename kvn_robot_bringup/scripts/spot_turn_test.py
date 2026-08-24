#!/usr/bin/env python3
"""
Session 5: Spot-Turn Test Script
Performs controlled spot-turn maneuvers and logs odometry vs IMU yaw for calibration.

Usage:
  ros2 run kvn_robot_bringup spot_turn_test.py

The script will:
1. Record data for N_TESTS spot-turn maneuvers
2. Each maneuver: rotate for ROTATION_DURATION seconds at ANGULAR_VELOCITY
3. Log odometry yaw and IMU yaw to CSV files
4. Calculate yaw error and effectiveness for calibration analysis
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import csv
import time
from datetime import datetime
import math


class SpotTurnTester(Node):
    """Test node for spot-turn calibration."""
    
    def __init__(self):
        super().__init__('spot_turn_tester')
        
        # Test parameters
        self.rotation_duration = 10.0  # seconds
        self.angular_velocity = 0.5    # rad/s (rotate in place)
        self.n_tests = 3               # number of test runs
        self.rest_duration = 5.0       # seconds between tests
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers for feedback
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.imu_callback, 10)
        
        # Data storage
        self.odom_data = []
        self.imu_data = []
        self.start_time = None
        self.test_start_time = None
        
        # Latest measurements
        self.latest_odom_yaw = 0.0
        self.latest_imu_yaw_rate = 0.0
        self.last_imu_yaw = 0.0
        self.integrated_imu_yaw = 0.0
        
        # Create results directory
        import os
        self.results_dir = os.path.expanduser('~/ros_kvn_ws/session5_results')
        os.makedirs(self.results_dir, exist_ok=True)
        
        self.get_logger().info(f'Spot-turn tester initialized. Results will be saved to {self.results_dir}')
    
    def odom_callback(self, msg):
        """Record odometry message with yaw angle."""
        if self.test_start_time is None:
            return  # Not recording yet
        
        # Extract yaw from quaternion (assuming small angles, approximate)
        q = msg.pose.pose.orientation
        # Simplified yaw calculation from quaternion
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        elapsed = (self.get_clock().now() - self.test_start_time).nanoseconds / 1e9
        
        self.odom_data.append({
            'time': elapsed,
            'yaw': yaw,
            'linear_x': msg.twist.twist.linear.x,
            'angular_z': msg.twist.twist.angular.z
        })
        
        self.latest_odom_yaw = yaw
    
    def imu_callback(self, msg):
        """Record IMU yaw rate."""
        if self.test_start_time is None:
            return  # Not recording yet
        
        # Store yaw rate directly
        yaw_rate = msg.angular_velocity.z
        
        elapsed = (self.get_clock().now() - self.test_start_time).nanoseconds / 1e9
        
        # Integrate yaw rate (trapezoidal rule)
        if self.imu_data:
            dt = elapsed - self.imu_data[-1]['time']
            if dt > 0:
                avg_rate = (self.imu_data[-1]['yaw_rate'] + yaw_rate) / 2.0
                self.integrated_imu_yaw += avg_rate * dt
        
        self.imu_data.append({
            'time': elapsed,
            'yaw_rate': yaw_rate,
            'integrated_yaw': self.integrated_imu_yaw
        })
        
        self.latest_imu_yaw_rate = yaw_rate
    
    def send_velocity_command(self, linear_x=0.0, angular_z=0.0):
        """Send velocity command to robot."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
    
    def run_test(self, test_number):
        """Run one spot-turn test."""
        self.get_logger().info(f'\n==== TEST {test_number}/{self.n_tests} ====')
        self.get_logger().info(f'Duration: {self.rotation_duration}s at {self.angular_velocity} rad/s')
        
        # Clear data
        self.odom_data = []
        self.imu_data = []
        self.integrated_imu_yaw = 0.0
        
        # Start recording
        self.test_start_time = self.get_clock().now()
        self.get_logger().info('Starting rotation...')
        
        # Send rotation command
        self.send_velocity_command(linear_x=0.0, angular_z=self.angular_velocity)
        
        # Let it rotate
        start = time.time()
        while (time.time() - start) < self.rotation_duration:
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)
        
        # Stop
        self.get_logger().info('Stopping rotation')
        self.send_velocity_command(linear_x=0.0, angular_z=0.0)
        time.sleep(1.0)
        
        # Save results
        self.save_test_data(test_number)
        
        # Rest between tests
        if test_number < self.n_tests:
            self.get_logger().info(f'Resting for {self.rest_duration}s before next test...')
            time.sleep(self.rest_duration)
    
    def save_test_data(self, test_number):
        """Save test data to CSV files."""
        # Filenames with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        odom_file = f'{self.results_dir}/test{test_number}_odom_{timestamp}.csv'
        imu_file = f'{self.results_dir}/test{test_number}_imu_{timestamp}.csv'
        
        # Save odometry data
        with open(odom_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['time', 'yaw', 'linear_x', 'angular_z'])
            writer.writeheader()
            writer.writerows(self.odom_data)
        
        # Save IMU data
        with open(imu_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['time', 'yaw_rate', 'integrated_yaw'])
            writer.writeheader()
            writer.writerows(self.imu_data)
        
        # Calculate metrics
        if len(self.odom_data) > 1 and len(self.imu_data) > 1:
            odom_yaw_final = self.odom_data[-1]['yaw']
            imu_yaw_final = self.imu_data[-1]['integrated_yaw']
            yaw_error = abs(odom_yaw_final - imu_yaw_final)
            yaw_error_pct = 100.0 * yaw_error / abs(imu_yaw_final) if imu_yaw_final != 0 else 0.0
            
            self.get_logger().info(f'Results saved to {odom_file} and {imu_file}')
            self.get_logger().info(f'Odometry final yaw: {odom_yaw_final:.3f} rad ({math.degrees(odom_yaw_final):.1f}°)')
            self.get_logger().info(f'IMU integrated yaw: {imu_yaw_final:.3f} rad ({math.degrees(imu_yaw_final):.1f}°)')
            self.get_logger().info(f'Yaw error: {yaw_error:.3f} rad ({yaw_error_pct:.1f}%)')
    
    def main(self):
        """Run all tests."""
        self.get_logger().info(f'Starting spot-turn test suite ({self.n_tests} tests)')
        self.get_logger().info(f'Angular velocity: {self.angular_velocity} rad/s')
        self.get_logger().info(f'Duration per test: {self.rotation_duration}s')
        
        for i in range(1, self.n_tests + 1):
            try:
                self.run_test(i)
            except KeyboardInterrupt:
                self.get_logger().warn('Test interrupted by user')
                break
        
        # Final stop
        self.send_velocity_command(linear_x=0.0, angular_z=0.0)
        self.get_logger().info('\nAll tests completed!')
        self.get_logger().info(f'Results saved to: {self.results_dir}')
        self.get_logger().info('Next: Run session5_analyze.ipynb to calculate optimal chi parameter')


def main(args=None):
    rclpy.init(args=args)
    node = SpotTurnTester()
    
    try:
        node.main()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.send_velocity_command(0.0, 0.0)  # Ensure motor stop
        rclpy.shutdown()


if __name__ == '__main__':
    main()
