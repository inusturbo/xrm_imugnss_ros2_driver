# WTGAHRS2 ROS2 Driver for WitMotion IMU/GNSS Sensor

This repository contains a ROS2 driver for the WTGAHRS2 IMU/GNSS sensor, manufactured by WitMotion. 
The driver has been adapted and modified from the [ElettraSciComp/witmotion_IMU_ros (ros2 branch)](https://github.com/ElettraSciComp/witmotion_IMU_ros/tree/ros2) and the [WITMOTION/WTGAHRS2](https://github.com/WITMOTION/WTGAHRS2) repositories.

## Features

- Seamless integration with ROS2 ecosystems
- Real-time communication with the WitMotion WTGAHRS2 IMU/GNSS sensor
- Publishes sensor data on appropriate ROS2 topics
- Supports various robotic platforms and applications

## Installation

### Prerequisites

The package requires QtSerialPort development package from Qt 5.2+

```bash
sudo apt-get install libqt5serialport5-dev
```

Clone the repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/XRMobility/xrm_imugnss_ros2_driver.git
```

Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select witmotion_ros
```

Source the setup file:

```bash
source install/setup.bash
```

## Usage

### Solution for CH340 Series Serial Port Driver Issues (no ttyUSB): 

If you connect the sensor and find that the output of the command `ls /dev/ttyUSB*` is empty, please execute the following command: `sudo apt remove brltty`. Now, unplug and replug the sensor's USB connection, and it should work.

Now connect the WitMotion WTGAHRS2 IMU/GNSS sensor to your computer.

Launch the driver using the provided launch file:

```bash
sudo chmod 777 /dev/ttyUSB0
ros2 launch witmotion_ros gnssimu.launch.py
```

The driver will publish the sensor data on appropriate ROS2 topics, which you can subscribe to and use in your applications.

You can adjust the message publishing Topic and the message publishing frequency from the config file.

## Configuration
Configuration of the node is done by default via the configuration YAML file [`config.yml`](./config/config.yml). But it also can be done using [`roslaunch` XML syntax](https://wiki.ros.org/roslaunch/XML) under the node's internal namespace. The single value measurements, like pressure and temperature, are enabled for the linear calibration because there can be differences in decoding coefficients between the sensors.

### Parameters
- `port` - the virtual kernel device name for a port, `ttyUSB0` by default
- `baud_rate` - port rate value to be used by the library for opening the port, _9600 baud_ by default
- `polling_interval` - the sensor polling interval in milliseconds. If this parameter is omitted, the default value is set up by the library (50 ms).
- `restart_service_name` - the service name used to restart the sensor connection after an error.
- `imu_publisher:`
    - `topic_name` - the topic name for IMU data publisher, `imu` in the node's namespace by default
    - `frame_id` - IMU message header [frame ID](https://wiki.ros.org/tf)
    - `use_native_orientation` - instructs the node to use the native quaternion orientation measurement from the sensor instead of synthesized from Euler angles. **NOTE**: if this setting is enabled bu the sensor does not produce orientation in the quaternion format, the IMU message will never be published!
    - `measurements` - every measurement in IMU message data pack can be enabled or disabled. If the measurement is disabled, the corresponding covariance matrix is set to begin from `-1` as it is described in the [message definition](https://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/Imu.html).
        - `acceleration`
            - `enabled`
            - `covariance` - row-major matrix 3x3, all zeros for unknown covariation
        - `angular_velocity`
            - `enabled`
            - `covariance` - row-major matrix 3x3, all zeros for unknown covariation
        - `orientation`
            - `enabled`
            - `covariance` - row-major matrix 3x3, all zeros for unknown covariation
- `temperature_publisher`
    - `enabled` - enable or disable temperature measurement extraction
    - `topic_name` - the topic name for publishing temperature data
    - `frame_id` - message header frame ID
    - `from_message` - the message type string to determine from which type of Witmotion measurement message the temperature data should be extracted (please refer to the original documentation for detailed description). The possible values are: `acceleration`, `angular_vel`, `orientation` or `magnetometer`.
    - `variance` - the constant variance, if applicable, otherwise 0
    - `coefficient` - linear calibration multiplier, 1.0 by default
    - `addition` - linear calibration addendum, 0 by default
- `magnetometer_publisher`
    - `enabled` - enable or disable magnetometer measurement extraction
    - `topic_name` - the topic name for publishing the data
    - `frame_id` - message header frame ID
    - `coefficient` - linear calibration multiplier, 1.0 by default
    - `addition` - linear calibration addendum, 0 by default
    - `covariance` - row-major matrix 3x3, all zeros for unknown covariation
- `barometer_publisher`
    - `enabled` - enable or disable barometer measurement extraction
    - `topic_name` - the topic name for publishing the data
    - `frame_id` - message header frame ID
    - `coefficient` - linear calibration multiplier, 1.0 by default
    - `addition` - linear calibration addendum, 0 by default
    - `variance` - the constant variance, if applicable, otherwise 0
- `altimeter_publisher`
    - `enabled` - enable or disable altitude measurement extraction
    - `topic_name` - the topic name for publishing the data
    - `coefficient` - linear calibration multiplier, 1.0 by default
    - `addition` - linear calibration addendum, 0 by default
- `orientation_publisher`
    - `enabled` - enable or disable orientation measurement extraction
    - `topic_name` - the topic name for publishing the data
- `gps_publisher`
    - `enabled` - enables/disables all GPS receiver measurements extraction
    - `navsat_fix_frame_id` - frame ID for GPS fixed position publisher
    - `navsat_fix_topic_name` - topic name for GPS fixed position publisher
    - `navsat_altitude_topic_name` - topic name for GPS altitude publisher
    - `navsat_satellites_topic_name` - topic name for GPS active satellites number publisher
    - `navsat_variance_topic_name` - topic name for GPS diagonal variance publisher
    - `ground_speed_topic_name` - topic name for GPS ground speed publisher
- `rtc_publisher`
    - `enabled` - enables/disables realtime clock information decoder
    - `topic_name` - topic name for realtime clock publisher
    - `presync` - instructs the node to perform an attempt to pre-synchronize sensor's internal realtime clock


## Acknowledgements

This driver is based on the work of ElettraSciComp and WITMOTION. We appreciate their contributions to the open-source community and their efforts in developing the original repositories.
