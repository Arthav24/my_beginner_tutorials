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
├── build/
├── install/
├── log/
└── src/
    ├── beginner_tutorials/
```
## Assignment 1 - Publisher/Subscriber

This is a tutorial package to learn pub-sub mechanism. <br>
Original codes can be found here https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

Package directory structure:

```
beginner_tutorials/
├── include/
└── src/
    ├── publisher_member_function.cpp
    ├── subscriber_member_function.cpp
```

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
colcon build --packages-select beginner_tutorials
```

## How to Run:
First, source the overlay:
```bash
source install/setup.bash
ros2 run beginner_tutorials talker # on one terminal 
ros2 run beginner_tutorials listner # on another terminal
```