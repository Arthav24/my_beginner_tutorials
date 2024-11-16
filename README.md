# Repository for ENPM 700 ROS2

### Dependencies/ Assumptions/ Tools used in project
- OS: Ubuntu:22.04
- Developed and tested on x86-64 architecture
- ROS2 - humble (https://docs.ros.org/en/humble/Installation.html)
- catch_ros2
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
│   │   ├── bag.launch.py
│   │   ├── demo.launch.py
│   │   └── integration_test.launch.yaml
│   ├── LICENSE
│   ├── package.xml
│   ├── src
│   │   ├── publisher_member_function.cc
│   │   └── subscriber_member_function.cc
│   └── test
│       └── integration_test_node.cc
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

 
# To test static tf broadcaster
ros2 run beginner_tutorials talker --ros-args -p freq:=10.0 #To publish tf
ros2 run tf2_ros tf2_echo world talk

# To run Level 2 integration test 
colcon test  --return-code-on-test-failure --event-handlers console_cohesion+ --packages-select beginner_tutorials

# To generate frames tree in gv or pdf 
ros2 run tf2_tools view_frames
```

## Output of Transformation from world to talk
```bash
At time 1731711605.634570712
- Translation: [10.000, -10.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, -0.650, 0.760]
- Rotation: in RPY (radian) [0.000, 0.000, -1.416]
- Rotation: in RPY (degree) [0.000, 0.000, -81.127]
- Matrix:
  0.154  0.988  0.000 10.000
 -0.988  0.154 -0.000 -10.000
 -0.000  0.000  1.000  0.000
  0.000  0.000  0.000  1.000

```

Original codes can be found here https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
