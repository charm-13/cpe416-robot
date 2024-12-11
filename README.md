# robot 2073
a.k.a roomba 2.0 (work in progress)<br>
The codebase for CPE416, Autonomous Mobile Robotics, final project.<br>

The code includes the following:
1) Our main ros2 driver. Found in `gobilda_ws/src/final_project/`,
2) The Gobilda ros2 driver. Found in `gobilda_ws/src/gobilda_robot/`,
3) The RPLIDAR A1 ros2 driver. Found in `gobilda_ws/src/rplidar_ros/`, and
4) The JETGPIO library. Found in `JETGPIO/` 

## Setup
### ROS Setup on the Orin:
Run the following to install ROS 2:
1) Set locale: 
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
2) Setup sources:
```
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
3) Install ROS2 standard packages:
```
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```
4) Install extra ROS2 packages:
Nav2:
```
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```
SLAM:
```
sudo apt install ros-humble-slam-toolbox
```

### Install JETGPIO library and change the orin's pin configuration.
1) To install the library, run the following in `cpe416-robot/JETGPIO`:
```sudo make```
```sudo make install```
2) To change the orin pin configuration, run the following:
```sudo /opt/nvidia/jetson-io/jetson-io.py```
This should bring up a curses app/menu that will allow you to configure the pins for different outputs.
3) Set pins 15 & 32 to be PWM.
4) Save the configuration and restart your Orin.

### Install all the dependencies that are listed for the packages. 
1) In `cpe416-robot/gobilda_ws/`, run:
```sudo rosdep init```
```rosdep update```
```rosdep install --from-paths src -yr --ignore-src```
2) Edit `src/gobilda_robot/description/urdf/gobilda_description.urdf.xacro` so that the description of the robot actually matches the physical placement of your robot and sensors.
3) Tune parameters in `src/gobilda_robot/bringup/config/gobilda_controllers.yaml` to match the robot.

## Launch robot 2073
1) Compile and source `gobilda_ws`<br>
i.e. `source /opt/ros/humble/setup.bash`, `colcon build`, `source install/setup.bash`
4) To interface with the JETGPIO library and use the funcs in the ros2_control_node you will need to be root. `sudo -i`
5) Run the launch file: `ros2 launch gobilda_robot gobilda.launch.py`.<br> 
NOTE: because the launch file was ran using root, the nodes can only communicate with other root nodes. You will need to be root again to run for example "ros2 topic list"

## Contributors
 - Charlotte Maples
 - Noah Masten
 - Emanuel Gonzales
