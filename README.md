# NavSatFix Publisher

This ROS2 package publishes NavSatFix messages with configurable GPS location and noise.

## Package Overview

The package contains:
- A publisher node that generates and publishes NavSatFix messages
- A configuration file to set GPS location, publish rate, and noise parameters
- A launch file to easily start the publisher with the configuration

## Configuration

The GPS parameters can be configured in `config/gps_config.yaml`:

- `latitude`: Initial latitude in degrees
- `longitude`: Initial longitude in degrees
- `altitude`: Initial altitude in meters
- `publish_rate`: Publishing frequency in Hz
- `noise_range`: Noise range in meters (applied to latitude, longitude, and altitude)

## Building and Running

1. Build the package:
   ```bash
   colcon build --packages-select navsatfix_pub
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Run the publisher:
   ```bash
   ros2 run navsatfix_pub navsatfix_publisher
   ```

   Or use the launch file:
   ```bash
   ros2 launch navsatfix_pub gps_publisher.launch.py
   ```

## Subscribed Topics

None

## Published Topics

- `/gps/fix` (sensor_msgs/msg/NavSatFix): GPS fix data with added noise

## Parameters

- `latitude` (double): Initial latitude in degrees
- `longitude` (double): Initial longitude in degrees
- `altitude` (double): Initial altitude in meters
- `publish_rate` (double): Publishing frequency in Hz
- `noise_range` (double): Noise range in meters