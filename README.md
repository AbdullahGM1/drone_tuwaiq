# Drone Tuwaiq - ROS2 Drone Control Package

A comprehensive ROS2 package for controlling drones in simulation using PX4 autopilot, Gazebo Harmonic simulator, and micro-XRCE-DDS communication bridge. Developed for Tuwaiq Academy drone programming course.

## 🚁 Package Overview

This package provides:
- **Drone Controller Node**: Core flight control and command processing
- **Command Interface Node**: Interactive command-line interface for drone control  
- **Launch System**: Automated startup of PX4 SITL, Gazebo simulation, and ROS2 bridge
- **Sensor Integration**: Camera, LiDAR, IMU, GPS, and pressure sensor support

## 📋 Prerequisites

### System Requirements
- **Operating System**: Ubuntu 24.04 LTS
- **ROS2 Distribution**: Jazzy Jalisco
- **Gazebo**: Harmonic
- **Python**: 3.12 or higher
- **Memory**: Minimum 8GB RAM (16GB recommended)
- **Storage**: At least 20GB free space

## 🛠️ Installation Guide

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

### 4. Configure Custom PX4 Models and Airframes

#### 4.1. Add Custom Models to PX4

```bash
# Navigate to PX4 models directory
cd ~/PX4-Autopilot/Tools/simulation/gz/models

# Add your custom models here (if you have any)
# Models should be in SDF format with proper directory structure
# Example:
# cp -r /path/to/your/custom_model ./

# List existing models
ls -la
```

#### 4.2. Configure Custom Airframe

```bash
# Navigate to PX4 airframes directory
cd ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes

# Check if airframe 4021 already exists and remove it if necessary
if [ -f "4021_gz_x500_lidar_camera" ]; then
    echo "Removing existing 4021 airframe..."
    rm 4021_gz_x500_lidar_camera
fi

# Create the new 4021_gz_x500_lidar_camera airframe file
cat > 4021_gz_x500_lidar_camera << 'EOF'
#!/bin/sh
#
# @name Gazebo X500 with LiDAR and Camera
# @type Quadrotor
#

. ${R}etc/init.d/rc.mc_defaults

param set-default CA_ROTOR_COUNT 4
param set-default CA_ROTOR0_PX 0.13
param set-default CA_ROTOR0_PY 0.22
param set-default CA_ROTOR1_PX -0.13
param set-default CA_ROTOR1_PY -0.22
param set-default CA_ROTOR2_PX 0.13
param set-default CA_ROTOR2_PY -0.22
param set-default CA_ROTOR3_PX -0.13
param set-default CA_ROTOR3_PY 0.22

param set-default CA_ROTOR0_KM -0.05
param set-default CA_ROTOR1_KM -0.05
param set-default CA_ROTOR2_KM 0.05
param set-default CA_ROTOR3_KM 0.05

param set-default PWM_MAIN_FUNC1 101
param set-default PWM_MAIN_FUNC2 102
param set-default PWM_MAIN_FUNC3 103
param set-default PWM_MAIN_FUNC4 104

param set-default SIM_GZ_EN 1
param set-default SIM_GZ_MODEL x500_lidar_camera

# Enable sensors
param set-default SENS_EN_GPSSIM 1
param set-default SENS_EN_BAROSIM 1
param set-default SENS_EN_MAGSIM 1

# Set GPS parameters
param set-default EKF2_GPS_CTRL 7

# Set magnetometer parameters
param set-default EKF2_MAG_TYPE 1

# Enable optical flow
param set-default EKF2_OF_CTRL 1

set MIXER quad_x
EOF

# Make the airframe file executable
chmod +x 4021_gz_x500_lidar_camera
```

#### 4.3. Update CMakeLists.txt

```bash
# Navigate to airframes directory
cd ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes

# Backup the original CMakeLists.txt
cp CMakeLists.txt CMakeLists.txt.backup

# Find the line with 4020 and add 4021 after it
sed -i '/4020_gz_x500_depth_camera/a\\t4021_gz_x500_lidar_camera' CMakeLists.txt

# Verify the changes
echo "Checking CMakeLists.txt for 4021 entry:"
grep -A2 -B2 "4021_gz_x500_lidar_camera" CMakeLists.txt
```

#### 4.4. Build PX4 with Custom Configuration

```bash
# Navigate to PX4 root directory
cd ~/PX4-Autopilot

# Clean previous builds (optional but recommended)
make clean

# Build PX4 SITL
make px4_sitl

# Test the new airframe
make px4_sitl gz_x500_lidar_camera
```

#### 4.5. Verify Installation

```bash
# List available airframes to confirm 4021 is included
cd ~/PX4-Autopilot
./build/px4_sitl_default/bin/px4 -s ROMFS/px4fmu_common/init.d-posix/airframes/4021_gz_x500_lidar_camera

# Check if the airframe loads correctly
echo "If no errors appear above, the airframe is configured correctly!"
```

### 5. Install micro-XRCE-DDS Agent

```bash
# Install dependencies
sudo apt install python3-pip build-essential cmake git -y

# Clone and build micro-XRCE-DDS-Agent
cd ~/
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

# Add to PATH (add to ~/.bashrc)
echo 'export PATH=$PATH:~/Micro-XRCE-DDS-Agent/build' >> ~/.bashrc
source ~/.bashrc
```

### 6. Install Additional ROS2 Jazzy Packages

```bash
# Install required ROS2 packages for Jazzy
sudo apt install ros-jazzy-geometry-msgs -y
sudo apt install ros-jazzy-sensor-msgs -y
sudo apt install ros-jazzy-std-msgs -y
sudo apt install ros-jazzy-nav-msgs -y
sudo apt install ros-jazzy-tf2-ros -y
sudo apt install ros-jazzy-tf2-geometry-msgs -y
sudo apt install ros-jazzy-ros-gz-bridge -y
sudo apt install ros-jazzy-ros-gz-sim -y

# Install development tools
sudo apt install python3-pytest -y
sudo apt install ros-jazzy-ament-lint -y
sudo apt install ros-jazzy-ament-cmake -y

# Verify ROS2 Jazzy packages
echo "ROS2 Jazzy packages installed successfully!"
ros2 pkg list | grep -E "(geometry_msgs|sensor_msgs|std_msgs)"
```

## 🏗️ Build Instructions

### 1. Create ROS2 Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this package
git clone <your-repository-url> drone_tuwaiq
```

### 2. Build the Package

```bash
cd ~/ros2_ws

# Source ROS2 Jazzy environment
source /opt/ros/jazzy/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select drone_tuwaiq

# Source the workspace
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## 🚀 Usage

### 1. Launch the Complete System

```bash
# Terminal 1: Launch the drone system
ros2 launch drone_tuwaiq drone_tuwaiq.launch.py
```

This will automatically start:
- PX4 SITL simulation
- Gazebo Harmonic with X500 drone model
- micro-XRCE-DDS agent
- ROS2-Gazebo bridge

### 2. Start the Drone Controller

```bash
# Terminal 2: Start drone controller
ros2 run drone_tuwaiq drone_controller
```

### 3. Use the Command Interface

```bash
# Terminal 3: Interactive command interface
ros2 run drone_tuwaiq drone_command

# Or send single command
ros2 run drone_tuwaiq drone_command takeoff
```

### 4. Manual PX4 Connection (if needed)

After PX4 shows "Ready for takeoff!", run in PX4 console:
```bash
uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888
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
drone_tuwaiq/
├── drone_tuwaiq/
│   ├── __init__.py
│   ├── drone_controller.py      # Main flight controller
│   └── command_interface.py     # Command interface
├── launch/
│   └── drone_tuwaiq.launch.py   # Launch file
├── package.xml                  # Package dependencies
├── setup.py                     # Setup configuration
└── README.md                    # This file
```

## 🔧 Troubleshooting

### Common Issues

1. **PX4 won't start**
   ```bash
   # Check if ports are in use
   sudo lsof -i :14570
   sudo lsof -i :8888
   
   # Kill existing processes
   pkill -f px4
   pkill -f MicroXRCEAgent
   ```

2. **Gazebo crashes**
   ```bash
   # Set Gazebo resource path for Harmonic
   export GZ_SIM_RESOURCE_PATH=/usr/share/gz
   
   # Reset Gazebo configuration
   rm -rf ~/.gz
   ```

3. **micro-XRCE-DDS connection fails**
   ```bash
   # Manually start the agent
   MicroXRCEAgent udp4 -p 8888
   
   # Check PX4 parameters
   param show UXRCE_DDS_CFG
   ```

4. **ROS2 topics not visible**
   ```bash
   # Source all environments
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   
   # Check bridge status
   ros2 topic list | grep drone
   ```

### Performance Optimization

1. **For better simulation performance:**
   ```bash
   # Reduce Gazebo real-time factor
   export GZ_SIM_RESOURCE_PATH=/usr/share/gz
   
   # Use headless mode for Gazebo
   gz sim -s --headless-rendering
   ```

2. **For development:**
   ```bash
   # Build in debug mode
   colcon build --packages-select drone_tuwaiq --cmake-args -DCMAKE_BUILD_TYPE=Debug
   
   # Enable logging
   export ROS_LOG_DIR=~/ros_logs
   ```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 👨‍💻 Author

**Abdullah GM** (AbdullahGM1)
- Email: agm.musalami@gmail.com

## 🔗 Related Resources

- [PX4 Documentation](https://docs.px4.io/)
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [micro-XRCE-DDS Documentation](https://micro-xrce-dds.docs.eprosima.com/)