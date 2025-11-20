# ESP32-S3 Camera & LiDAR Sensor Platform

Multi-sensor platform for ESP32-S3 combining optical flow and distance measurement for robotics and navigation applications.

## Features

- **Optical Flow Measurement**: Track velocity using camera-based or dedicated sensors
- **Distance Measurement**: Accurate ranging with multiple LiDAR options
- **Dual-Core Architecture**: FreeRTOS tasks optimized for real-time performance
- **Binary Protocol**: High-speed UART output with timestamp, velocity (X/Y), and distance data

## Supported Sensors

### Optical Flow
- **Cameras**: OV2640, OV5640, OV7725 (configurable resolution and parameters)
- **PMW3901**: Dedicated optical flow sensor (121 FPS, 42° FOV)
- **MTF-02**: Integrated distance + optical flow sensor

### LiDAR/Distance
- **TFMini**: 115200 baud, up to 12m range
- **DTS6012M**: 921600 baud high-speed sensor
- **MTF-02**: Combined distance and optical flow measurement

## Configuration

Edit `main/main.c` to select sensors by uncommenting the appropriate defines:
- Choose one optical flow mode: `USE_CAMERA`, `USE_PMW3901`, or `LIDAR_ONLY_MODE`
- Choose one camera (if using `USE_CAMERA`): `USE_OV2640`, `USE_OV5640`, or `USE_OV7725`
- Choose one LiDAR: `USE_TFMINI`, `USE_DTS6012M`, or `USE_MTF02`

## Output Protocol

Binary packet format: `[0xAA 0x55][timestamp(4)][velocity_x(2)][velocity_y(2)][distance(2)][checksum(1)]`
