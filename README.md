# 🚁 Drone Tuwaiq - Advanced ROS2 Drone Control System

<div align="center">

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-orange.svg)](https://px4.io/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-green.svg)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.12+-yellow.svg)](https://python.org/)
[![License](https://img.shields.io/badge/License-MIT-red.svg)](LICENSE)

*A comprehensive ROS2 package for advanced drone simulation and control using PX4 autopilot, Gazebo Harmonic simulator, and micro-XRCE-DDS communication bridge. Developed for Tuwaiq Academy drone programming course.*

</div>

---

## ✨ Features

### 🎯 Core Capabilities
- **🤖 Intelligent Drone Controller**: Advanced flight control with state management
- **🎮 Interactive Command Interface**: User-friendly CLI for drone operation
- **🚀 Automated Launch System**: One-command startup for complete simulation environment
- **📡 Multi-Sensor Integration**: Camera, LiDAR, IMU, GPS, and pressure sensors
- **🔗 PX4 Integration**: Native PX4 message support with micro-XRCE-DDS bridge
- **🌍 Custom Gazebo Models**: X500 quadcopter with gimbal camera and LiDAR

### 🎛️ Advanced Features
- **📊 Real-time Status Monitoring**: Live drone state feedback
- **🎯 Precision Flight Control**: Velocity-based movement commands
- **🔄 State Machine Management**: Intelligent flight state transitions
- **🛡️ Safety Systems**: Automatic failsafe and emergency procedures
- **📈 Extensible Architecture**: Easy integration of custom sensors and controllers

## 🏗️ System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Command       │    │   Drone         │    │   PX4 SITL      │
│   Interface     │────│   Controller    │────│   Simulation    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                 ┌─────────────────────────────┐
                 │    micro-XRCE-DDS Agent     │
                 │         (Bridge)            │
                 └─────────────────────────────┘
                                 │
                 ┌─────────────────────────────┐
                 │     Gazebo Harmonic         │
                 │    (3D Simulation)          │
                 └─────────────────────────────┘
```

## 📋 Prerequisites

### 🖥️ System Requirements
- **Operating System**: Ubuntu 24.04 LTS (Noble Numbat)
- **ROS2 Distribution**: Jazzy Jalisco
- **Gazebo**: Harmonic (latest)
- **Python**: 3.12 or higher
- **Memory**: Minimum 8GB RAM (16GB recommended for smooth operation)
- **Storage**: At least 25GB free space
- **GPU**: Dedicated GPU recommended for optimal Gazebo performance

### 🔧 Software Dependencies
- **PX4 Autopilot**: Latest stable version
- **micro-XRCE-DDS Agent**: Communication bridge
- **ROS2 Jazzy packages**: Core and additional packages
- **Gazebo Harmonic**: 3D simulation environment

## 🛠️ Complete Installation Guide

### 1. Install ROS2 Jazzy on Ubuntu 24.04

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Set locale
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y

# Add ROS2 GPG key with new method (apt-key is deprecated)
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS2 Jazzy Desktop
sudo apt update
sudo apt install ros-jazzy-desktop -y

# Install additional ROS2 tools
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep python3-pip -y
sudo apt install python3-argcomplete -y

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS2 environment
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
echo "ROS2 Jazzy installation completed!"
ros2 --version
```

### 2. Install Gazebo Harmonic

```bash
# Install required dependencies
sudo apt-get update
sudo apt-get install lsb-release wget gnupg -y

# Add Gazebo official repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic
sudo apt-get update
sudo apt-get install gz-harmonic -y

# Install ROS2-Gazebo bridge for Jazzy
sudo apt install ros-jazzy-ros-gz-bridge -y
sudo apt install ros-jazzy-ros-gz-sim -y
```

### 3. Install PX4 Autopilot

```bash
# Clone PX4 repository
cd ~/
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Install PX4 dependencies
bash ./Tools/setup/ubuntu.sh

# Build PX4 for SITL
make px4_sitl

# Test PX4 installation
make px4_sitl gz_x500
```

### 4. 🔧 Configure Custom PX4 Models and Airframes

> **🎯 Important**: This section configures the custom X500 drone model with LiDAR and camera integration

#### 4.1. 📁 Setup Custom Gazebo Models

```bash
# Navigate to PX4 models directory
cd ~/PX4-Autopilot/Tools/simulation/gz/models

# Copy custom models from the drone_tuwaiq package
# (This will be done during package installation)
echo "✅ Custom models will be integrated during package build"

# Verify existing models
ls -la | grep x500
```

#### 4.2. ⚙️ Configure Custom Airframe (4021_gz_x500_lidar_camera)

```bash
# Navigate to PX4 airframes directory
cd ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes

# Check if airframe 4021 already exists and backup if necessary
if [ -f "4021_gz_x500_lidar_camera" ]; then
    echo "🔄 Backing up existing 4021 airframe..."
    mv 4021_gz_x500_lidar_camera 4021_gz_x500_lidar_camera.backup
fi

# Create the new 4021_gz_x500_lidar_camera airframe file with advanced configuration
cat > 4021_gz_x500_lidar_camera << 'EOF'
#!/bin/sh
# @name Gazebo X500 with LiDAR and Camera
# @type Quadrotor
# @class Copter

. ${R}etc/init.d/rc.mc_defaults

# Gazebo and simulation settings
PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500_lidar_camera}

param set-default SIM_GZ_EN 1

# micro-XRCE-DDS configuration
param set-default UXRCE_DDS_DOM_ID 0
param set-default UXRCE_DDS_CFG 0

# Sensor configurations
param set-default SENS_EN_GPSSIM 1
param set-default SENS_EN_BAROSIM 1
param set-default SENS_EN_MAGSIM 1

# IMU configuration for simulation
param set-default IMU_GYRO_RATEMAX 2000
param set-default SENS_IMU_MODE 1
param set-default SIM_IMU_PUB_RATE 400

# System parameters
param set-default SYS_HAS_GPS 1
param set-default SYS_USE_IO 0

# EKF2 parameters for reliable position estimation
param set-default EKF2_AID_MASK 1
param set-default EKF2_HGT_MODE 0
param set-default EKF2_GPS_CTRL 7
param set-default EKF2_MAG_TYPE 1

# Control allocation for quad-X configuration
param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 4

# Motor positions (X configuration)
param set-default CA_ROTOR0_PX 0.13
param set-default CA_ROTOR0_PY 0.22
param set-default CA_ROTOR0_KM 0.05

param set-default CA_ROTOR1_PX -0.13
param set-default CA_ROTOR1_PY -0.20
param set-default CA_ROTOR1_KM 0.05

param set-default CA_ROTOR2_PX 0.13
param set-default CA_ROTOR2_PY -0.22
param set-default CA_ROTOR2_KM -0.05

param set-default CA_ROTOR3_PX -0.13
param set-default CA_ROTOR3_PY 0.20
param set-default CA_ROTOR3_KM -0.05

# ESC/Motor configurations
param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_FUNC3 103
param set-default SIM_GZ_EC_FUNC4 104

# ESC limits
param set-default SIM_GZ_EC_MIN1 150
param set-default SIM_GZ_EC_MIN2 150
param set-default SIM_GZ_EC_MIN3 150
param set-default SIM_GZ_EC_MIN4 150

param set-default SIM_GZ_EC_MAX1 1000
param set-default SIM_GZ_EC_MAX2 1000
param set-default SIM_GZ_EC_MAX3 1000
param set-default SIM_GZ_EC_MAX4 1000

# Flight performance tuning
param set-default MPC_THR_HOVER 0.60
EOF

# Make the airframe file executable
chmod +x 4021_gz_x500_lidar_camera
echo "✅ Custom airframe 4021_gz_x500_lidar_camera created successfully!"
```

#### 4.3. 📝 Update CMakeLists.txt

```bash
# Navigate to airframes directory
cd ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes

# Backup the original CMakeLists.txt
cp CMakeLists.txt CMakeLists.txt.backup.$(date +%Y%m%d_%H%M%S)

# Add the new airframe to CMakeLists.txt
if ! grep -q "4021_gz_x500_lidar_camera" CMakeLists.txt; then
    sed -i '/4020_gz_x500_depth_camera/a\\t4021_gz_x500_lidar_camera' CMakeLists.txt
    echo "✅ Added 4021_gz_x500_lidar_camera to CMakeLists.txt"
else
    echo "ℹ️  4021_gz_x500_lidar_camera already exists in CMakeLists.txt"
fi

# Verify the changes
echo "🔍 Checking CMakeLists.txt for 4021 entry:"
grep -A2 -B2 "4021_gz_x500_lidar_camera" CMakeLists.txt || echo "❌ Entry not found!"
```

#### 4.4. 🔨 Build PX4 with Custom Configuration

```bash
# Navigate to PX4 root directory
cd ~/PX4-Autopilot

# Clean previous builds (recommended for configuration changes)
echo "🧹 Cleaning previous builds..."
make distclean

# Build PX4 SITL with our custom configuration
echo "🔨 Building PX4 SITL..."
make px4_sitl

# Verify the build completed successfully
if [ $? -eq 0 ]; then
    echo "✅ PX4 SITL build completed successfully!"
else
    echo "❌ PX4 SITL build failed. Please check the error messages above."
    exit 1
fi
```

#### 4.5. ✅ Verify Custom Airframe Installation

```bash
# Test the new airframe configuration
echo "🧪 Testing custom airframe configuration..."

# Check if the airframe file is executable and readable
if [ -x ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4021_gz_x500_lidar_camera ]; then
    echo "✅ Airframe file is properly configured and executable"
else
    echo "❌ Airframe file has permission issues"
fi

# Quick syntax check of the airframe file
cd ~/PX4-Autopilot
bash -n ROMFS/px4fmu_common/init.d-posix/airframes/4021_gz_x500_lidar_camera
if [ $? -eq 0 ]; then
    echo "✅ Airframe syntax is valid"
else
    echo "❌ Airframe syntax has errors"
fi

echo "🚀 Custom airframe setup completed!"
echo "   Next: Install micro-XRCE-DDS Agent for communication bridge"
```

### 5. 🔗 Install micro-XRCE-DDS Agent (Communication Bridge)

> **📡 Critical Component**: This agent bridges PX4 and ROS2 communications

```bash
# Install build dependencies
echo "📦 Installing build dependencies..."
sudo apt update
sudo apt install python3-pip build-essential cmake git libasio-dev libtinyxml2-dev -y

# Clone the official micro-XRCE-DDS-Agent repository
echo "📥 Cloning micro-XRCE-DDS-Agent..."
cd ~/
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent

# Create and configure build directory
echo "🔨 Building micro-XRCE-DDS-Agent..."
mkdir build && cd build

# Configure with cmake (with optimizations)
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build with all available cores
make -j$(nproc)

# Install system-wide
sudo make install

# Update library cache
sudo ldconfig

# Add to PATH for easy access
echo 'export PATH=$PATH:~/Micro-XRCE-DDS-Agent/build' >> ~/.bashrc
echo 'export MICRO_XRCE_DDS_AGENT_PATH=~/Micro-XRCE-DDS-Agent/build' >> ~/.bashrc
source ~/.bashrc

# Verify installation
echo "✅ Verifying micro-XRCE-DDS-Agent installation..."
if command -v MicroXRCEAgent &> /dev/null; then
    echo "✅ MicroXRCEAgent is available in PATH"
    MicroXRCEAgent --version || echo "ℹ️  Agent installed (version info not available)"
else
    echo "❌ MicroXRCEAgent not found in PATH"
    echo "   Manual path: ~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent"
fi
```

### 6. 📦 Install PX4 Message Definitions and Additional ROS2 Packages

> **🔧 Essential**: Install all required ROS2 packages and PX4 message definitions

```bash
# Install core ROS2 Jazzy packages
echo "📦 Installing core ROS2 Jazzy packages..."
sudo apt update
sudo apt install -y \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-interfaces

# Install development and testing tools
echo "🛠️ Installing development tools..."
sudo apt install -y \
    python3-pytest \
    ros-jazzy-ament-lint \
    ros-jazzy-ament-cmake \
    ros-jazzy-launch-xml \
    ros-jazzy-launch-yaml \
    ros-jazzy-xacro

# Build and install PX4 messages for ROS2
echo "📡 Setting up PX4 message definitions..."
cd ~/
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs

# Switch to the appropriate branch for ROS2 Jazzy
git checkout ros2

# Build the PX4 messages package
cd ~/
mkdir -p px4_msgs_ws/src
cd px4_msgs_ws/src
cp -r ~/px4_msgs ./
cd ..

# Source ROS2 and build
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs

# Add PX4 messages to ROS2 environment
echo "source ~/px4_msgs_ws/install/setup.bash" >> ~/.bashrc

# Verify package installations
echo "✅ Verifying ROS2 package installations..."
source ~/.bashrc
ros2 pkg list | grep -E "(geometry_msgs|sensor_msgs|std_msgs|px4_msgs)" | head -10

# Test PX4 message availability
echo "🧪 Testing PX4 message definitions..."
if ros2 interface list | grep -q px4_msgs; then
    echo "✅ PX4 messages are available!"
    echo "📋 Available PX4 message types:"
    ros2 interface list | grep px4_msgs | head -5
else
    echo "⚠️  PX4 messages need to be built manually"
    echo "   Run: cd ~/px4_msgs_ws && colcon build --packages-select px4_msgs"
fi
```

### 7. 🌐 Setup Environment Variables and Configuration

```bash
# Create a comprehensive environment setup script
echo "⚙️ Creating environment configuration..."
cat > ~/.drone_tuwaiq_env << 'EOF'
#!/bin/bash
# Drone Tuwaiq Environment Configuration

# ROS2 Jazzy setup
source /opt/ros/jazzy/setup.bash

# PX4 messages setup
if [ -f ~/px4_msgs_ws/install/setup.bash ]; then
    source ~/px4_msgs_ws/install/setup.bash
fi

# Workspace setup (will be added during package installation)
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Gazebo Harmonic environment
export GZ_SIM_RESOURCE_PATH=/usr/share/gz:$GZ_SIM_RESOURCE_PATH

# PX4 environment
export PX4_HOME=~/PX4-Autopilot

# micro-XRCE-DDS Agent
export MICRO_XRCE_DDS_AGENT_PATH=~/Micro-XRCE-DDS-Agent/build

# Performance optimizations
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/PX4-Autopilot/Tools/simulation/gz/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/PX4-Autopilot/build/px4_sitl_default/build_gazebo

echo "🚁 Drone Tuwaiq environment loaded!"
EOF

# Add environment loader to bashrc
echo "source ~/.drone_tuwaiq_env" >> ~/.bashrc

# Source the environment
source ~/.drone_tuwaiq_env

echo "✅ Environment configuration completed!"
```

## 🏗️ Build Instructions

### 1. 📁 Create and Setup ROS2 Workspace

```bash
# Create the main workspace directory
echo "📁 Creating ROS2 workspace..."
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Initialize the workspace
echo "🔧 Initializing workspace..."
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. 📥 Clone and Setup the Drone Tuwaiq Package

```bash
# Navigate to source directory
cd ~/ros2_ws/src

# Clone the drone_tuwaiq package
echo "📥 Cloning drone_tuwaiq package..."
# Replace with your actual repository URL
# git clone <your-repository-url> drone_tuwaiq

# For now, copy from current location (if available)
# cp -r /path/to/current/drone_tuwaiq ./

echo "✅ Package source ready"
```

### 3. 🔨 Build the Package

> **⚡ Performance Tip**: Use parallel compilation for faster builds

```bash
cd ~/ros2_ws

# Source all environments
source ~/.drone_tuwaiq_env

# Install package dependencies using rosdep
echo "📦 Installing package dependencies..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the drone_tuwaiq package with optimizations
echo "🔨 Building drone_tuwaiq package..."
colcon build \
    --packages-select drone_tuwaiq \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers $(nproc)

# Check build status
if [ $? -eq 0 ]; then
    echo "✅ Package built successfully!"
else
    echo "❌ Build failed. Check error messages above."
    exit 1
fi

# Source the workspace
source install/setup.bash

# Add workspace to environment permanently
if ! grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi
```

### 4. 🧪 Verify Installation

```bash
# Test package installation
echo "🧪 Verifying package installation..."

# Check if package is recognized by ROS2
if ros2 pkg list | grep -q drone_tuwaiq; then
    echo "✅ drone_tuwaiq package found in ROS2"
else
    echo "❌ drone_tuwaiq package not found"
    exit 1
fi

# Check executables
echo "🔍 Checking available executables:"
ros2 pkg executables drone_tuwaiq

# Check launch files
echo "🚀 Checking launch files:"
ros2 launch drone_tuwaiq --show-args || echo "Launch file verification..."

# Test nodes can be imported
echo "🐍 Testing Python node imports..."
python3 -c "
try:
    import rclpy
    from drone_tuwaiq.drone_controller import DroneController
    from drone_tuwaiq.command_interface import CommandInterface
    print('✅ All Python modules imported successfully')
except ImportError as e:
    print(f'❌ Import error: {e}')
"

echo "🎉 Installation verification completed!"
```

### 5. 🔧 Setup Custom Models in Gazebo

```bash
# Copy custom models to PX4 models directory
echo "📁 Setting up custom Gazebo models..."

# Create models directory if it doesn't exist
mkdir -p ~/PX4-Autopilot/Tools/simulation/gz/models

# Copy models from the package
cp -r ~/ros2_ws/src/drone_tuwaiq/models/* ~/PX4-Autopilot/Tools/simulation/gz/models/

# Copy world files
mkdir -p ~/PX4-Autopilot/Tools/simulation/gz/worlds
cp ~/ros2_ws/src/drone_tuwaiq/world/*.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/

# Copy PX4 airframe configuration
cp ~/ros2_ws/src/drone_tuwaiq/px4/4021_gz_x500_lidar_camera \
   ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/

# Make airframe executable
chmod +x ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4021_gz_x500_lidar_camera

echo "✅ Custom models and configurations installed"
```

## 🚀 Usage Guide

### 1. 🎬 Complete System Startup

> **📋 Recommended Workflow**: Follow this step-by-step process for optimal results

#### Terminal 1: Launch the Simulation Environment

```bash
# Source the environment
source ~/.drone_tuwaiq_env

# Launch the complete simulation stack
ros2 launch drone_tuwaiq drone_tuwaiq.launch.py

# Expected output sequence:
# 1. PX4 SITL starts and loads airframe 4021
# 2. Gazebo Harmonic opens with X500 drone model
# 3. micro-XRCE-DDS agent starts on port 8888
# 4. ROS2-Gazebo bridge establishes communication
# 5. PX4 shows "Ready for takeoff!"
```

#### Terminal 2: Start the Drone Controller

```bash
# Wait for PX4 to show "Ready for takeoff!" before running this
source ~/.drone_tuwaiq_env

# Start the intelligent drone controller
ros2 run drone_tuwaiq drone_controller

# Expected output:
# ✅ Drone Controller Node Started
# 📡 Available commands: arm, disarm, takeoff, land, up, down, forward, backward, left, right, stop
```

#### Terminal 3: Connect PX4 to ROS2 (Manual Step)

```bash
# In the PX4 console (Terminal 1), type this command:
uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888

# Expected output in PX4:
# [INFO] XRCE_DDS_CLIENT: Running...
# [INFO] XRCE_DDS_CLIENT: Connected to agent
```

#### Terminal 4: Interactive Command Interface

```bash
source ~/.drone_tuwaiq_env

# Method 1: Interactive mode (recommended for learning)
ros2 run drone_tuwaiq drone_command

# Method 2: Single command execution
ros2 run drone_tuwaiq drone_command takeoff
ros2 run drone_tuwaiq drone_command "move up"
ros2 run drone_tuwaiq drone_command land
```

### 2. 🎯 Flight Operations Workflow

#### Basic Flight Sequence

```bash
# 1. Arm the drone
ros2 run drone_tuwaiq drone_command arm

# 2. Wait for arming confirmation, then takeoff
ros2 run drone_tuwaiq drone_command takeoff

# 3. Perform flight maneuvers
ros2 run drone_tuwaiq drone_command forward
ros2 run drone_tuwaiq drone_command up
ros2 run drone_tuwaiq drone_command right

# 4. Stop and hover
ros2 run drone_tuwaiq drone_command stop

# 5. Land safely
ros2 run drone_tuwaiq drone_command land

# 6. Disarm after landing
ros2 run drone_tuwaiq drone_command disarm
```

#### Advanced Interactive Session

```bash
# Start interactive mode
ros2 run drone_tuwaiq drone_command

# Example interactive session:
🎮 Enter command: help          # Show all available commands
🎮 Enter command: arm           # Arm the motors
🎮 Enter command: takeoff       # Takeoff to hover altitude
🎮 Enter command: forward       # Move forward
🎮 Enter command: up            # Gain altitude
🎮 Enter command: stop          # Stop and hover
🎮 Enter command: land          # Land the drone
🎮 Enter command: quit          # Exit interface
```

### 3. 📊 System Monitoring

#### Real-time Status Monitoring

```bash
# Monitor drone status (in a separate terminal)
ros2 topic echo /drone/command

# Monitor PX4 vehicle status
ros2 topic echo /fmu/out/vehicle_status

# Monitor sensor data
ros2 topic echo /drone/imu           # IMU data
ros2 topic echo /drone/gps           # GPS position
ros2 topic echo /drone/scan          # LiDAR data

# Monitor camera feed
ros2 topic echo /drone/gimbal_camera --no-arr  # Camera images (metadata only)
```

#### System Health Checks

```bash
# Check all active topics
ros2 topic list | grep -E "(drone|fmu)"

# Verify micro-XRCE-DDS bridge is working
ros2 node list | grep -i xrce

# Monitor system resources
htop  # Check CPU and memory usage
```

### 4. 🛠️ Troubleshooting During Operation

#### If PX4 Connection Fails

```bash
# Check if micro-XRCE-DDS agent is running
ps aux | grep MicroXRCEAgent

# Restart the agent if needed
pkill MicroXRCEAgent
MicroXRCEAgent udp4 -p 8888

# In PX4 console, reconnect
uxrce_dds_client stop
uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888
```

#### If Gazebo Becomes Unresponsive

```bash
# Gracefully restart Gazebo
pkill gz
sleep 2
ros2 launch drone_tuwaiq drone_tuwaiq.launch.py
```

#### Reset Complete System

```bash
# Kill all processes
pkill -f px4
pkill MicroXRCEAgent
pkill gz
pkill ros2

# Wait and restart
sleep 3
ros2 launch drone_tuwaiq drone_tuwaiq.launch.py
```

## 🎮 Available Commands

### Basic Commands
- `arm` - Arm the drone motors
- `disarm` - Disarm the drone motors
- `takeoff` - Takeoff to hover altitude
- `land` - Land the drone

### Movement Commands
- `up` - Move up (Z+)
- `down` - Move down (Z-)
- `forward` - Move forward (X+)
- `backward` - Move backward (X-)
- `left` - Move left (Y+)
- `right` - Move right (Y-)
- `stop` - Stop all movement

### Interface Commands
- `help` - Show command help
- `quit` - Exit command interface

## 📡 ROS2 Topics

### Published Topics
- `/drone/arm` - Arm command
- `/drone/disarm` - Disarm command  
- `/drone/takeoff` - Takeoff command
- `/drone/land` - Land command
- `/fmu/in/setpoint_velocity` - Velocity commands to PX4

### Subscribed Topics
- `/drone/command` - String commands from interface

### Available Sensor Topics
- `/drone/gimbal_camera` - Camera image
- `/drone/gimbal_camera_info` - Camera info
- `/drone/scan` - LiDAR scan data
- `/drone/scan/points` - LiDAR point cloud
- `/drone/imu` - IMU data
- `/drone/gps` - GPS position
- `/drone/air_pressure` - Pressure sensor

## 📁 Package Structure

```
📦 drone_tuwaiq/
├── 🐍 drone_tuwaiq/                    # Python package source
│   ├── __init__.py                     # Package initialization
│   ├── drone_controller.py             # 🤖 Main flight controller node
│   └── command_interface.py            # 🎮 Interactive command interface
├── 🚀 launch/                          # ROS2 launch files
│   ├── drone_tuwaiq.launch.py          # 🎬 Main system launcher
│   └── notes.txt                       # 📝 Development notes
├── 🌍 models/                          # Gazebo model definitions
│   ├── gimbal_small_3d/                # 📷 Camera gimbal model
│   │   ├── model.config                # Model configuration
│   │   ├── model.sdf                   # SDF model definition
│   │   └── meshes/                     # 3D mesh files
│   ├── lidar/                          # 📡 LiDAR sensor model
│   │   ├── model.config
│   │   └── model.sdf
│   └── x500_lidar_camera/              # 🚁 Complete drone model
│       ├── model.config                # X500 with sensors
│       └── model.sdf
├── 🌐 world/                           # Gazebo world definitions
│   └── default.sdf                     # Default simulation world
├── ✈️ px4/                             # PX4 configurations
│   └── 4021_gz_x500_lidar_camera       # Custom airframe definition
├── 📋 package.xml                      # ROS2 package dependencies
├── ⚙️ setup.py                         # Python package setup
├── 🏷️ resource/                        # ROS2 resource files
│   └── drone_tuwaiq                    # Package marker
└── 📖 README.md                        # This comprehensive guide
```

### 🔧 Core Components

#### 🤖 Drone Controller (`drone_controller.py`)
- **State Management**: Intelligent flight state tracking (IDLE, ARMED, TAKEOFF, HOVERING, LANDING, etc.)
- **Command Processing**: Handles arm/disarm, takeoff/land, and movement commands
- **Velocity Control**: Publishes precise velocity commands to `/fmu/in/setpoint_velocity`
- **Safety Features**: Automatic state transitions and emergency handling
- **Real-time Feedback**: Continuous status updates and logging

#### 🎮 Command Interface (`command_interface.py`)
- **Interactive Mode**: User-friendly CLI with real-time command input
- **Single Command Mode**: Execute individual commands from terminal
- **Help System**: Built-in command reference and usage examples
- **Error Handling**: Graceful handling of invalid commands and connection issues

#### 🚀 Launch System (`drone_tuwaiq.launch.py`)
- **Automated Startup**: One-command launch of entire simulation stack
- **Service Coordination**: Proper timing and dependencies between services
- **Bridge Configuration**: Complete ROS2-Gazebo topic mapping
- **Environment Setup**: Gazebo world loading and model initialization

#### 🌍 Custom Models
- **X500 Drone**: Realistic quadcopter physics and dynamics
- **LiDAR Integration**: 360° laser scanning capability
- **Gimbal Camera**: 3-axis stabilized camera system
- **Sensor Suite**: IMU, GPS, pressure sensors, and more

## 🔧 Comprehensive Troubleshooting Guide

### 🚨 Common Issues and Solutions

#### 1. 🔌 PX4 Connection Issues

**Problem**: PX4 SITL won't start or shows connection errors

```bash
# 🔍 Diagnosis: Check if ports are in use
sudo lsof -i :14570  # PX4 MAVLink port
sudo lsof -i :8888   # micro-XRCE-DDS port

# 🧹 Solution: Clean up existing processes
pkill -f px4
pkill -f MicroXRCEAgent
pkill -f gazebo
pkill -f gz

# 🔄 Restart with clean slate
sleep 3
ros2 launch drone_tuwaiq drone_tuwaiq.launch.py
```

#### 2. 🎮 Gazebo Simulation Problems

**Problem**: Gazebo crashes, freezes, or shows rendering issues

```bash
# 🔍 Check Gazebo configuration
echo $GZ_SIM_RESOURCE_PATH

# 🧹 Reset Gazebo configuration
rm -rf ~/.gz
rm -rf ~/.gazebo

# 🌐 Set proper environment variables
export GZ_SIM_RESOURCE_PATH=/usr/share/gz:~/PX4-Autopilot/Tools/simulation/gz/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/PX4-Autopilot/Tools/simulation/gz/models

# 🖥️ For systems with GPU issues, use software rendering
export LIBGL_ALWAYS_SOFTWARE=1

# 💨 For better performance on low-end systems
gz sim --verbose --headless-rendering
```

#### 3. 📡 micro-XRCE-DDS Communication Failures

**Problem**: ROS2 topics not appearing, no PX4 communication

```bash
# 🔍 Check if agent is running
ps aux | grep MicroXRCEAgent

# 🔄 Restart the agent
pkill MicroXRCEAgent
sleep 1
MicroXRCEAgent udp4 -p 8888 -v 6  # Verbose mode for debugging

# 🔧 In PX4 console, check and restart client
param show UXRCE_DDS_CFG
uxrce_dds_client stop
sleep 1
uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888
```

#### 4. 🤖 ROS2 Topic and Node Issues

**Problem**: ROS2 topics not visible, nodes not starting

```bash
# 🌐 Source all required environments
source /opt/ros/jazzy/setup.bash
source ~/.drone_tuwaiq_env

# 🔍 Verify package installation
ros2 pkg list | grep drone_tuwaiq

# 🧪 Test node startup individually
ros2 run drone_tuwaiq drone_controller --ros-args --log-level DEBUG

# 📊 Check topic bridge status
ros2 topic list | grep -E "(drone|fmu)"
ros2 node list
```

#### 5. 🏗️ Build and Compilation Errors

**Problem**: Package won't build, import errors, dependency issues

```bash
# 🧹 Clean workspace
cd ~/ros2_ws
rm -rf build/ install/ log/

# 📦 Update dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 🔨 Rebuild with verbose output
colcon build --packages-select drone_tuwaiq --cmake-args -DCMAKE_BUILD_TYPE=Debug --verbose

# 🐍 Test Python imports manually
python3 -c "
import sys
sys.path.append('~/ros2_ws/install/drone_tuwaiq/lib/python3.12/site-packages')
from drone_tuwaiq.drone_controller import DroneController
print('✅ Import successful')
"
```

### ⚡ Performance Optimization

#### 🚀 Simulation Performance

```bash
# 🎯 Optimize Gazebo for your hardware
export GZ_SIM_RESOURCE_PATH=/usr/share/gz
export GAZEBO_MASTER_URI=http://localhost:11345

# 💻 For systems with limited resources
export GZ_SIM_SERVER_CONFIG_PATH=~/gazebo_server.config
gz sim -s --headless-rendering --iterations 1000

# 🔧 Reduce physics update rate (in world file or runtime)
# Physics update: 1000 Hz -> 250 Hz for better performance
```

#### 🏃 ROS2 Node Performance

```bash
# 🔄 Increase node processing rate
export ROS_DOMAIN_ID=0
export RCUTILS_LOGGING_SEVERITY=WARN  # Reduce log verbosity

# 🧮 Multi-threaded executor for better performance
# (Implemented in the drone_controller.py automatically)
```

#### 🖥️ Development and Debugging

```bash
# 📋 Enable detailed logging
export ROS_LOG_DIR=~/ros_logs
export RCUTILS_LOGGING_SEVERITY=DEBUG

# 🔍 Build with debugging symbols
colcon build --packages-select drone_tuwaiq \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# 📊 Performance monitoring
htop  # Monitor CPU and RAM usage
ros2 topic hz /drone/command  # Check topic update rates
```

### 🧪 System Health Checks

#### Daily Verification Script

```bash
#!/bin/bash
# 🏥 drone_tuwaiq_health_check.sh

echo "🔍 Running Drone Tuwaiq System Health Check..."

# Check ROS2 installation
echo "📦 Checking ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash && echo "✅ ROS2 OK" || echo "❌ ROS2 Failed"

# Check PX4
echo "🚁 Checking PX4 Autopilot..."
[ -d ~/PX4-Autopilot ] && echo "✅ PX4 Directory OK" || echo "❌ PX4 Missing"

# Check micro-XRCE-DDS
echo "📡 Checking micro-XRCE-DDS Agent..."
command -v MicroXRCEAgent >/dev/null && echo "✅ Agent OK" || echo "❌ Agent Missing"

# Check Gazebo
echo "🌍 Checking Gazebo Harmonic..."
gz --version >/dev/null 2>&1 && echo "✅ Gazebo OK" || echo "❌ Gazebo Failed"

# Check workspace
echo "🏗️ Checking workspace..."
[ -f ~/ros2_ws/install/setup.bash ] && echo "✅ Workspace OK" || echo "❌ Workspace Missing"

echo "🎉 Health check completed!"
```

#### 🔧 Quick Reset Script

```bash
#!/bin/bash
# 🔄 drone_tuwaiq_reset.sh

echo "🔄 Resetting Drone Tuwaiq System..."

# Kill all processes
pkill -f px4
pkill MicroXRCEAgent  
pkill gz
pkill gazebo

# Wait for cleanup
sleep 3

# Restart core services
echo "🚀 Restarting simulation..."
cd ~/ros2_ws
source ~/.drone_tuwaiq_env
ros2 launch drone_tuwaiq drone_tuwaiq.launch.py

echo "✅ System reset completed!"
```

## 🎓 Educational Resources

### 📚 Learning Path for Students

#### 🌟 Beginner Level
1. **ROS2 Fundamentals**
   - Understanding nodes, topics, services, and actions
   - Basic ROS2 CLI commands and tools
   - Simple publisher/subscriber patterns

2. **PX4 Basics**
   - Drone flight modes and controls
   - MAVLink communication protocol
   - Basic SITL simulation concepts

3. **Gazebo Introduction**
   - 3D simulation environment basics
   - Model and world creation
   - Sensor simulation concepts

#### 🚀 Intermediate Level
1. **Advanced ROS2 Concepts**
   - Launch files and parameters
   - Custom message types
   - Service and action servers
   - Multi-node systems

2. **Drone Programming**
   - Flight control algorithms
   - Sensor data processing
   - Navigation and path planning
   - Computer vision integration

3. **Simulation Integration**
   - Custom Gazebo models
   - Physics parameter tuning
   - Multi-robot simulations

#### 🎯 Advanced Level
1. **Professional Development**
   - Real hardware integration
   - Safety systems and failsafes
   - Performance optimization
   - Production deployment

2. **Research Applications**
   - SLAM (Simultaneous Localization and Mapping)
   - Autonomous navigation
   - Swarm robotics
   - AI/ML integration

### 📖 Recommended Reading

- **📘 "Programming Robots with ROS" by Morgan Quigley**
- **📗 "Learning ROS for Robotics Programming" by Aaron Martinez**
- **📙 "Autonomous Flying Robots" by Kenzo Nonami**
- **📕 "Gazebo Simulation" documentation and tutorials**

## 🤝 Contributing to Drone Tuwaiq

### 🎯 How to Contribute

We welcome contributions from the community! Here's how you can help:

#### 🐛 Bug Reports
1. Check existing issues first
2. Provide detailed reproduction steps
3. Include system information and logs
4. Use the bug report template

#### ✨ Feature Requests
1. Describe the proposed feature clearly
2. Explain the use case and benefits
3. Consider backward compatibility
4. Follow the feature request template

#### 🔧 Code Contributions

1. **Fork and Clone**
   ```bash
   git clone https://github.com/your-username/drone_tuwaiq.git
   cd drone_tuwaiq
   ```

2. **Create Feature Branch**
   ```bash
   git checkout -b feature/amazing-new-feature
   ```

3. **Make Changes**
   - Follow coding standards (PEP 8 for Python)
   - Add comprehensive documentation
   - Include unit tests where applicable
   - Update README if needed

4. **Test Thoroughly**
   ```bash
   # Build and test your changes
   cd ~/ros2_ws
   colcon build --packages-select drone_tuwaiq
   
   # Run the simulation and test all functionality
   ros2 launch drone_tuwaiq drone_tuwaiq.launch.py
   ```

5. **Submit Pull Request**
   - Provide clear description of changes
   - Reference any related issues
   - Ensure CI tests pass

### 📋 Development Guidelines

#### 🐍 Python Code Standards
- Follow PEP 8 style guidelines
- Use type hints where appropriate
- Add docstrings for functions and classes
- Keep functions small and focused

#### 📝 Documentation Standards
- Update README for new features
- Add inline code comments
- Include usage examples
- Update troubleshooting guide if needed

#### 🧪 Testing Requirements
- Test on Ubuntu 24.04 with ROS2 Jazzy
- Verify simulation works end-to-end
- Test both interactive and programmatic usage
- Check resource usage and performance

## 📜 License

This project is licensed under the **MIT License**.

```
MIT License

Copyright (c) 2024 Abdullah GM (AbdullahGM1)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## 👨‍💻 Author & Maintainer

<div align="center">

**Abdullah GM** ([@AbdullahGM1](https://github.com/AbdullahGM1))

*Senior Robotics Engineer & Educator*

📧 **Contact**: [agm.musalami@gmail.com](mailto:agm.musalami@gmail.com)

🎓 **Affiliation**: Tuwaiq Academy - Drone Programming Course

</div>

### 🙏 Acknowledgments

- **Tuwaiq Academy** for providing the educational platform and resources
- **PX4 Development Team** for the excellent autopilot software
- **ROS2 Community** for the robust robotics framework  
- **Gazebo Team** for the powerful simulation environment
- **eProsima** for the micro-XRCE-DDS communication bridge
- **Open Source Community** for continuous improvements and feedback

## 🔗 Related Resources & Documentation

### 📖 Official Documentation
- **[PX4 User Guide](https://docs.px4.io/)** - Complete PX4 autopilot documentation
- **[ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)** - Official ROS2 Jazzy docs
- **[Gazebo Harmonic Guide](https://gazebosim.org/docs/harmonic)** - Gazebo simulation documentation
- **[micro-XRCE-DDS Manual](https://micro-xrce-dds.docs.eprosima.com/)** - Communication bridge docs

### 🎓 Educational Resources  
- **[ROS2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)** - Step-by-step learning path
- **[PX4 Development Guide](https://dev.px4.io/)** - Developer documentation
- **[Gazebo Tutorials](https://gazebosim.org/docs/harmonic/tutorials)** - Simulation tutorials
- **[MAVLink Protocol](https://mavlink.io/en/)** - Drone communication standard

### 🛠️ Tools & Utilities
- **[QGroundControl](https://qgroundControl.com/)** - Ground control station
- **[ROS2 CLI Tools](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html)** - Command line interface
- **[Gazebo Model Database](https://app.gazebosim.org/models)** - Community models

### 🤖 Related Projects
- **[PX4-ROS2 Bridge](https://github.com/PX4/px4_ros_com)** - Official PX4-ROS2 integration
- **[MAVSDK](https://mavsdk.mavlink.io/)** - Drone API and SDK
- **[ArduPilot](https://ardupilot.org/)** - Alternative autopilot software

---

<div align="center">

**🚁 Happy Flying with Drone Tuwaiq! 🚁**

*Built with ❤️ for the robotics and drone community*

[![GitHub stars](https://img.shields.io/github/stars/AbdullahGM1/drone_tuwaiq?style=social)](https://github.com/AbdullahGM1/drone_tuwaiq)
[![GitHub forks](https://img.shields.io/github/forks/AbdullahGM1/drone_tuwaiq?style=social)](https://github.com/AbdullahGM1/drone_tuwaiq)

</div>