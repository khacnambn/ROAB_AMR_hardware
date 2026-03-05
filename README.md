# ROAB AMR Hardware

This repository contains the hardware firmware for the ROAB Autonomous Mobile Robot (AMR), developed for integration with ROS2 and Micro-ROS on an ESP32-S3 microcontroller.

## Overview

1. **PlatformIO Configuration (platform.ini)**  
   The `platform.ini` file defines the build environment. Saving this file triggers PlatformIO to build the code. It includes:  
   - Hardware definitions and communication port configurations.  
   - `lib_deps`: Required pre-installed libraries.  
   - Related dependencies.

2. **Custom Libraries**  
   Custom libraries are stored as header files in the `lib` folder.

3. **Configuration Constants**  
   Pin definitions and constant parameters used in the firmware are stored in the `config` folder.

## Running the Firmware

### Testing Procedure

1. **Upload Code to ESP32-S3**  
   Note the connection order for the ports:  
   - Connect the port farthest from the LED to `/dev/ttyACM0` first.  
   - Connect the port nearest to the LED to `/dev/ttyACM1` second.  
   This ensures proper upload and data reading.

2. **Run Micro-ROS on Terminals**  
   - **Terminal 1**: Start the Micro-ROS agent:
     
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1

   - **Terminal 2**: Control the motors using keyboard teleoperation:
  
   ros2 run teleop_twist_keyboard teleop_twist_keyboard

   - **Terminal 3**: List available ROS topics for verification:

   ros2 topic list

Expected topics:
/cmd_vel
/imu/data
/odom/unfiltered
/parameter_events
/rosout
