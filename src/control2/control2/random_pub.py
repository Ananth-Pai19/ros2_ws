#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from std_msgs.msg import Header, Float32
import random


class RandomGPSPublisher(Node):
    def __init__(self):
        super().__init__('random_gps_publisher')
        
        # Create publishers
        self.gps_pub = self.create_publisher(NavSatFix, '/gps_topic', 10)
        self.latency_pub = self.create_publisher(Float32, '/latency_topic', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu_topic', 10)

        # Base coordinates
        self.base_lat = 12.04
        self.base_lon = 13.99
        
        # Base latency (50ms) and yaw (60 degrees)
        self.base_latency = 50.0
        self.base_yaw = 60.0
        
        # Publish at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_stuff)
        
    
    def publish_stuff(self):
        # Publish GPS
        gps_msg = NavSatFix()
        
        # Header
        gps_msg.header = Header()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'gps'
        
        # Status
        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        
        # Add random noise to coordinates (±0.01 degrees ~ ±1.1 km)
        gps_msg.latitude = self.base_lat + random.uniform(-0.001, 0.001)
        gps_msg.longitude = self.base_lon + random.uniform(-0.001, 0.001)
        gps_msg.altitude = 50.0 + random.uniform(-1.0, 1.0)
        
        # Covariance (diagonal elements for uncertainty)
        gps_msg.position_covariance = [1.0, 0.0, 0.0,
                                        0.0, 1.0, 0.0,
                                        0.0, 0.0, 1.0]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.gps_pub.publish(gps_msg)
        
        # Publish Latency (around 50ms with ±5ms noise)
        latency_msg = Float32()
        latency_msg.data = self.base_latency + random.uniform(-5.0, 5.0)
        self.latency_pub.publish(latency_msg)
        
        # Publish Yaw (around 60 degrees with ±2 degree noise)
        imu_msg = Imu()
        imu_msg.orientation.z = self.base_yaw + random.uniform(-2.0, 2.0)
        self.imu_pub.publish(imu_msg)
        
        self.get_logger().info(
            f'GPS: lat={gps_msg.latitude:.6f}, lon={gps_msg.longitude:.6f} | '
            f'Latency: {latency_msg.data:.2f}ms | Yaw: {imu_msg.orientation.z:.2f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RandomGPSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
