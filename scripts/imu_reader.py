#!/usr/bin/env python3
"""
IMU Data Reader Node for ROS 2 Dashing
Đọc và xử lý dữ liệu từ IMU sensor
imu_reader sẽ đọc dữ liệu từ IMU, sau đó dữ liệu này được node ekf_localization sử dụng để kết hợp với Wheel Odometry, 
giúp robot định vị chính xác hơn khi di chuyển trong môi trường mô phỏng
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from rclpy.qos import qos_profile_sensor_data
import math

class IMUReader(Node):
    def __init__(self):
        super().__init__('imu_reader')
        
        # Subscriber cho IMU data
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            qos_profile_sensor_data
        )
        
        # Publisher cho orientation (góc Euler)
        self.euler_publisher = self.create_publisher(
            Vector3,
            'imu/euler',
            10
        )
        
        # Variables để lưu trữ dữ liệu
        self.orientation = None
        self.angular_velocity = None
        self.linear_acceleration = None
        
        self.get_logger().info('IMU Reader Node started!')
        self.get_logger().info('Subscribing to: /imu/data')
        self.get_logger().info('Publishing to: /imu/euler')

    def quaternion_to_euler(self, x, y, z, w):
        """
        Chuyển đổi từ Quaternion sang góc Euler (roll, pitch, yaw)
        
        Args:
            x, y, z, w: Quaternion components
            
        Returns:
            tuple: (roll, pitch, yaw) in radians
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def imu_callback(self, msg):
        """
        Callback function khi nhận được dữ liệu IMU
        
        Args:
            msg: sensor_msgs/Imu message
        """
        # Lưu dữ liệu
        self.orientation = msg.orientation
        self.angular_velocity = msg.angular_velocity
        self.linear_acceleration = msg.linear_acceleration
        
        # Chuyển đổi Quaternion sang Euler
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        
        # Chuyển đổi sang độ
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        # Publish Euler angles
        euler_msg = Vector3()
        euler_msg.x = roll
        euler_msg.y = pitch
        euler_msg.z = yaw
        self.euler_publisher.publish(euler_msg)
        
        # In thông tin (có thể comment out nếu không cần)
        # self.get_logger().info(
        #     f'\n'
        #     f'=== IMU Data ===\n'
        #     f'Orientation (Quaternion):\n'
        #     f'  x: {msg.orientation.x:.4f}\n'
        #     f'  y: {msg.orientation.y:.4f}\n'
        #     f'  z: {msg.orientation.z:.4f}\n'
        #     f'  w: {msg.orientation.w:.4f}\n'
        #     f'Orientation (Euler - degrees):\n'
        #     f'  Roll:  {roll_deg:.2f}°\n'
        #     f'  Pitch: {pitch_deg:.2f}°\n'
        #     f'  Yaw:   {yaw_deg:.2f}°\n'
        #     f'Angular Velocity (rad/s):\n'
        #     f'  x: {msg.angular_velocity.x:.4f}\n'
        #     f'  y: {msg.angular_velocity.y:.4f}\n'
        #     f'  z: {msg.angular_velocity.z:.4f}\n'
        #     f'Linear Acceleration (m/s²):\n'
        #     f'  x: {msg.linear_acceleration.x:.4f}\n'
        #     f'  y: {msg.linear_acceleration.y:.4f}\n'
        #     f'  z: {msg.linear_acceleration.z:.4f}\n'
        #     f'================'
        # )

def main(args=None):
    rclpy.init(args=args)
    
    imu_reader = IMUReader()
    
    try:
        rclpy.spin(imu_reader)
    except KeyboardInterrupt:
        pass
    
    imu_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
