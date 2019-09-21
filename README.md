# PLY ROS publisher

ROS node to publish a ply file, mainly for RVIZ visualization.

# 1. Installation

## A. Prerequisities

- [ROS](https://www.ros.org/install/)
- [PCL](docs.pointclouds.org)

## B. Installation

```bash
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init

# Add workspace to bashrc.
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

# Clone repo
cd ~/catkin_ws/src
git clone https://github.com/ToniRV/ply_ros_publisher.git

# Install dependencies from rosinstall file using wstool
wstool init
wstool merge ply_ros_publisher/install/ply_ros_publisher.rosinstall
wstool update
```

Finally, compile:

```bash
# Compile code
catkin build

# Refresh workspace
source ~/.bashrc
```

# 2. Usage

## rosrun
```
rosrun ply_ros_publisher ply_ros_publisher
```

## roslaunch
```
roslaunch ply_ros_publisher ply_ros_publisher
```

## rosservice
