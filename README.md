# realflight_ws

This is a real flight workspace for Learning to Coordinate experiments. This repository uses ROS2 Humble with PX4-v1.16 for real flight tests.

## Overview

This workspace provides the necessary infrastructure and tools for conducting real flight experiments as part of the Learning to Coordinate research. The system integrates ROS2 Humble with PX4-v1.16 autopilot firmware to enable autonomous drone coordination and control.

## Prerequisites

Before setting up this workspace, ensure you have the following installed:

- **ROS2 Humble**: Robot Operating System 2 (Humble Hawksbill distribution)
- **PX4-v1.16**: PX4 Autopilot firmware version 1.16
- **Ubuntu 22.04**: Recommended operating system for ROS2 Humble
- **Python 3.10+**: Required for ROS2 Humble
- **Colcon**: ROS2 build tool

## Installation

### 1. Install ROS2 Humble

Follow the official ROS2 Humble installation guide:
```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop
```

### 2. Install PX4-v1.16

Clone and build PX4 Autopilot:
```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.16.0
make px4_sitl_default
```

### 3. Clone and Build Workspace

```bash
# Clone this repository
cd ~
git clone https://github.com/CarlsonGYC/realflight_ws.git
cd realflight_ws

# Source ROS2
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Usage

### Setup Environment

Before running any commands, source both ROS2 and the workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/realflight_ws/install/setup.bash
```

### Running Flight Experiments

(To be documented as packages are added to the workspace)

## Project Structure

```
realflight_ws/
├── src/              # Source code for ROS2 packages
├── build/            # Build artifacts (generated)
├── install/          # Installation files (generated)
├── log/              # Build and runtime logs (generated)
└── README.md         # This file
```

## Dependencies

- **ROS2 Humble**: Core framework for robot software development
- **PX4-v1.16**: Flight control software for autonomous vehicles
- **Ubuntu 22.04**: Operating system

## Contributing

This is a research workspace for Learning to Coordinate experiments. Please contact the repository maintainer for contribution guidelines.

## License

(To be specified)

## Contact

For questions or issues, please open an issue on the GitHub repository.

## Acknowledgments

This workspace is part of the Learning to Coordinate research project.
