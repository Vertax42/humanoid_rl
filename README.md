# Humanoid_rl

Reinforcement learning inference ros package for Xbot-L humanoid robot.

## Directory Structure

```bash
humanoid_rl/src
├── bag                         # Record bag files
├── CMakeLists.txt              # Top LevelCMakeLists.txt
├── humanoid_rl                 # Humanoid RL package
│   ├── CMakeLists.txt          # CMakeLists.txt for humanoid_rl
│   ├── config                  # Config files for humanoid_rl
│   ├── include                 # Header files for humanoid_rl
│   ├── launch                  # Launch files for humanoid_rl
│   ├── model                   # Model files for humanoid_rl
│   ├── package.xml             # Package.xml for humanoid_rl
│   ├── scripts                 # Script files for humanoid_rl
│   └── src                     # Source files for humanoid_rl
├── joy_stick                   # Joy Stick package
│   ├── CMakeLists.txt          # CMakeLists.txt for joy_stick
│   ├── config                  # Config files for joy_stick
│   ├── include                 # Header files for joy_stick
│   ├── launch                  # Launch files for joy_stick
│   ├── package.xml             # Package.xml for joy_stick
│   ├── README.md               # README.md for joy_stick
│   └── src                     # Source files for joy_stick
└── robot_action_msg            # Robot Action Message package
    ├── CMakeLists.txt          # CMakeLists.txt for robot_action_msg
    ├── msg                     # Message files for robot_action_msg
    └── package.xml             # Package.xml for robot_action_msg
```

## Dependencies

Tested on Ubuntu 20.04, ROS Noetic.
gcc/g++ version: 9.4.0
CMake version: 3.16.3
libtorch version: 1.13.1+cpu
onnxruntime version: 1.8.1
libyaml version: 0.2.2
Eigen3 version: 3.3.7
Boost version: 1.71.0

## Installation

### Install dependencies

Install libyaml-dev

```bash
sudo apt-get install libyaml-dev
```

Install libtorch: Download from [libtorch website](https://pytorch.org/)

Install onnxruntime:

```bash
sudo apt update
sudo apt install -y build-essential cmake git libprotobuf-dev protobuf-compiler

git clone --recursive https://github.com/microsoft/onnxruntime

cd onnxruntime
./build.sh --config Release --build_shared_lib --parallel

cd build/Linux/Release/
sudo make install
