#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
import yaml
import random
import math

class NavSatFixPublisher(Node):
    def __init__(self):
        super().__init__('navsatfix_publisher')
        
        # Declare parameters with default values
        self.declare_parameter('latitude', 37.7749)
        self.declare_parameter('longitude', -122.4194)
        self.declare_parameter('altitude', 10.0)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('noise_range', 2.0)
        
        # Get parameters
        self.latitude = self.get_parameter('latitude').get_parameter_value().double_value
        self.longitude = self.get_parameter('longitude').get_parameter_value().double_value
        self.altitude = self.get_parameter('altitude').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.noise_range = self.get_parameter('noise_range').get_parameter_value().double_value
        
        # Create publisher
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        
        # Create timer
        timer_period = 1.0 / self.publish_rate  # Convert Hz to seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'NavSatFix Publisher started with rate {self.publish_rate} Hz')

    def add_noise_to_coordinates(self, lat, lon, alt, noise_range):
        """
        Add noise to GPS coordinates.
        Noise range is in meters.
        """
        # Earth's radius in meters
        earth_radius = 6378137.0
        
        # Convert noise in meters to degrees (approximation)
        # 1 degree latitude is approximately 111,320 meters
        noise_lat_deg = noise_range / 111320.0
        noise_lon_deg = noise_range / (111320.0 * math.cos(math.radians(lat)))
        
        # Add random noise
        noisy_lat = lat + random.uniform(-noise_lat_deg, noise_lat_deg)
        noisy_lon = lon + random.uniform(-noise_lon_deg, noise_lon_deg)
        noisy_alt = alt + random.uniform(-noise_range, noise_range)
        
        return noisy_lat, noisy_lon, noisy_alt

    def timer_callback(self):
        # Create NavSatFix message
        msg = NavSatFix()
        
        # Set header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        
        # Add noise to coordinates
        noisy_lat, noisy_lon, noisy_alt = self.add_noise_to_coordinates(
            self.latitude, self.longitude, self.altitude, self.noise_range)
        
        # Set position
        msg.latitude = noisy_lat
        msg.longitude = noisy_lon
        msg.altitude = noisy_alt
        
        # Set status (assuming GPS fix is good)
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        
        # Set position covariance (optional, simplified)
        # In a real application, this would be based on actual GPS accuracy
        covariance = [0.0] * 9
        covariance[0] = self.noise_range * self.noise_range  # Position variance in x
        covariance[4] = self.noise_range * self.noise_range  # Position variance in y
        covariance[8] = self.noise_range * self.noise_range  # Position variance in z
        msg.position_covariance = covariance
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        # Publish message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = NavSatFixPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()