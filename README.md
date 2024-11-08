# Repository for ENPM 700 ROS2

### Dependencies/ Assumptions/ Tools used in project
- OS: Ubuntu:22.04
- Developed and tested on x86-64 architecture
- ROS2 - humble (https://docs.ros.org/en/humble/Installation.html)
- python3-colcon-clean (sudo apt-get install python3-colcon-clean -y)
- terminator 
- Editor used - VSCODE (https://code.visualstudio.com/)

### Repository directory structure:

```
src/
├── beginner_tutorial_interfaces
│   ├── CMakeLists.txt
│   ├── include
│   │   └── beginner_tutorial_interfaces
│   ├── LICENSE
│   ├── package.xml
│   ├── src
│   └── srv
│       └── String.srv
├── beginner_tutorials
│   ├── clang_format_op.txt
│   ├── CMakeLists.txt
│   ├── cpplint_op.txt
│   ├── include
│   │   └── beginner_tutorials
│   ├── launch
│   │   └── demo.launch.py
│   ├── LICENSE
│   ├── package.xml
│   └── src
│       ├── publisher_member_function.cc
│       └── subscriber_member_function.cc
└── images
    └── rqt.png
```
## Directory setup

``` bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/Arthav24/my_beginner_tutorials.git
```

Install required dependencies

``` bash 
rosdep update
rosdep install --from-path src --ignore-src -y # assuming ROS_DISTRO is set to humble
```

## How to Compile:
```bash
cd ~/colcon_ws/
colcon clean workspace
source /opt/ros/humble/setup.bash  # if needed
colcon build
```

## How to Run:

```bash
source install/setup.bash
ros2 run beginner_tutorials talker --ros-args -p freq:=10.0 # on one terminal 
ros2 run beginner_tutorials listner # on another terminal

# To change base output string via service
ros2 service call /change_msg beginner_tutorial_interfaces/srv/String "{data: 'ENPM700'}"

# To launch as a system
ros2 launch beginner_tutorials demo.launch.py freq:=0.5

```

Original codes can be found here https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
