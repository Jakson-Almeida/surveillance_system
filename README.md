# Surveillance System Package

A comprehensive ROS package for autonomous surveillance using the P3DX robot platform with integrated camera systems and computer vision capabilities.

## Overview

The Surveillance System package provides a complete solution for autonomous surveillance operations, including:

- **Autonomous Patrol**: Predefined or random patrol patterns
- **Computer Vision**: Motion detection, face detection, and image processing
- **Alert System**: Real-time alerts for detected events
- **Recording System**: Automatic recording of surveillance footage
- **Navigation**: Autonomous navigation with obstacle avoidance
- **Status Monitoring**: System health and battery monitoring

## Features

### 🤖 Robot Control
- Autonomous patrol with configurable waypoints
- Obstacle detection and avoidance
- Emergency stop functionality
- Return-to-home capability

### 📹 Camera System
- Real-time image processing
- Motion detection algorithms
- Face detection using OpenCV
- Image enhancement and filtering
- Multi-camera support

### 🔍 Detection Capabilities
- Motion detection with configurable sensitivity
- Face detection using Haar cascades
- Extensible detection framework
- Confidence-based alerting

### 📊 Monitoring & Alerts
- Real-time status monitoring
- Alert system with configurable thresholds
- System health monitoring
- Battery level tracking

### 💾 Recording System
- Automatic video recording
- Configurable recording parameters
- Compression and storage management
- Event-triggered recording

## Package Structure

```
surveillance_system/
├── launch/                    # Launch files
│   ├── surveillance_system.launch    # Main system launch
│   ├── cameras.launch               # Camera setup
│   └── surveillance_control.launch  # Control system
├── scripts/                   # Python scripts
│   ├── surveillance_controller.py   # Main controller
│   ├── image_processor.py           # Image processing
│   ├── patrol_system.py             # Patrol management
│   ├── alert_system.py              # Alert handling
│   ├── recording_system.py          # Video recording
│   ├── navigation_controller.py     # Navigation control
│   └── status_monitor.py            # System monitoring
├── config/                    # Configuration files
│   └── patrol_points.yaml           # Patrol waypoints
├── worlds/                    # Gazebo world files
├── models/                    # 3D models
├── recordings/                # Recorded footage
└── msg/                       # Custom messages
```

## Dependencies

### ROS Packages
- `rospy` - Python client library
- `roscpp` - C++ client library
- `std_msgs` - Standard messages
- `sensor_msgs` - Sensor messages
- `geometry_msgs` - Geometry messages
- `nav_msgs` - Navigation messages
- `tf2` - Transform library
- `tf2_ros` - TF2 ROS interface
- `cv_bridge` - OpenCV bridge
- `image_transport` - Image transport

### System Dependencies
- OpenCV 4.x
- NumPy
- Python 3.x
- Gazebo (for simulation)

## Installation

1. **Clone the package** into your catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone <repository-url> surveillance_system
   ```

2. **Install dependencies**:
   ```bash
   sudo apt-get install ros-noetic-cv-bridge ros-noetic-image-transport
   pip3 install opencv-python numpy
   ```

3. **Build the package**:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## Usage

### Quick Start

1. **Launch the complete surveillance system**:
   ```bash
   roslaunch surveillance_system surveillance_system.launch
   ```

2. **Launch only the camera system**:
   ```bash
   roslaunch surveillance_system cameras.launch
   ```

3. **Launch only the control system**:
   ```bash
   roslaunch surveillance_system surveillance_control.launch
   ```

### Configuration

#### Patrol Points
Edit `config/patrol_points.yaml` to customize patrol routes:
```yaml
patrol_points:
  point_1:
    x: 5.0
    y: 0.0
    z: 0.0
    yaw: 0.0
    wait_time: 10.0
```

#### Camera Parameters
Modify camera settings in launch files:
```xml
<param name="frame_rate" value="30"/>
<param name="image_width" value="640"/>
<param name="image_height" value="480"/>
```

### Commands

Send commands to the surveillance system via ROS topics:

```bash
# Start patrol
rostopic pub /surveillance/commands std_msgs/String "start_patrol"

# Stop patrol
rostopic pub /surveillance/commands std_msgs/String "stop_patrol"

# Emergency stop
rostopic pub /surveillance/commands std_msgs/String "emergency_stop"

# Return home
rostopic pub /surveillance/commands std_msgs/String "return_home"
```

### Monitoring

#### View System Status
```bash
rostopic echo /surveillance/status
```

#### View Alerts
```bash
rostopic echo /surveillance/alerts
```

#### View Detections
```bash
rostopic echo /surveillance/detections
```

## Topics

### Subscribed Topics
- `/odom` - Robot odometry
- `/scan` - Laser scan data
- `/camera/image_raw` - Raw camera images
- `/camera/camera_info` - Camera calibration
- `/surveillance/commands` - Control commands

### Published Topics
- `/cmd_vel` - Robot velocity commands
- `/move_base_simple/goal` - Navigation goals
- `/processed_image` - Processed camera images
- `/surveillance/alerts` - System alerts
- `/surveillance/status` - System status
- `/surveillance/detections` - Detection results

## Services

### Available Services
- `/surveillance/get_status` - Get system status
- `/surveillance/set_mode` - Set surveillance mode
- `/surveillance/configure_patrol` - Configure patrol parameters

## Parameters

### Main Parameters
- `robot_speed` - Robot movement speed (default: 0.5 m/s)
- `patrol_radius` - Patrol area radius (default: 10.0 m)
- `detection_threshold` - Detection sensitivity (default: 0.7)
- `alert_cooldown` - Alert cooldown period (default: 5.0 s)

### Camera Parameters
- `frame_rate` - Camera frame rate (default: 30 fps)
- `image_width` - Image width (default: 640)
- `image_height` - Image height (default: 480)
- `motion_detection` - Enable motion detection (default: true)
- `face_detection` - Enable face detection (default: true)

## Customization

### Adding New Detection Algorithms
1. Extend the `ImageProcessor` class
2. Implement your detection method
3. Add configuration parameters
4. Update launch files

### Custom Patrol Patterns
1. Modify `patrol_points.yaml`
2. Implement custom patrol logic in `patrol_system.py`
3. Add new patrol modes

### Integration with External Systems
- Use ROS services for external control
- Publish alerts to external monitoring systems
- Integrate with security systems via ROS topics

## Troubleshooting

### Common Issues

1. **Camera not detected**:
   - Check camera connections
   - Verify camera drivers
   - Check topic names in launch files

2. **Robot not moving**:
   - Verify `/cmd_vel` topic is published
   - Check robot model configuration
   - Ensure navigation stack is running

3. **Detection not working**:
   - Check OpenCV installation
   - Verify camera topics are active
   - Check detection parameters

### Debug Mode
Enable debug output:
```bash
roslaunch surveillance_system surveillance_system.launch debug:=true
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For support and questions:
- Create an issue on GitHub
- Check the ROS wiki documentation
- Contact the maintainers

## Changelog

### Version 0.1.0
- Initial release
- Basic surveillance functionality
- P3DX robot integration
- Camera system support
- Motion and face detection
- Autonomous patrol system 