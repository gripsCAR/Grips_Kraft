grips
=====

ROS packages developed by the [Group of Robots and Intelligent Machines](http://www.romin.upm.es/) from the [Universidad Politécnica de Madrid](http://www.upm.es/internacional). This group is part of the [Centre for Automation and Robotics](http://www.car.upm-csic.es/) (CAR UPM-CSIC). On going development continues in the hydro-devel branch.

**Maintainer:** Francisco Suárez Ruiz, [http://www.romin.upm.es/fsuarez/](http://www.romin.upm.es/fsuarez/)

# Documentation
* See the installation instructions below.
* This repository.
* Throughout the various files in the packages.
* For questions, please use [http://answers.ros.org](http://answers.ros.org)

### Build Status

[![Build Status](https://travis-ci.org/fsuarez6/grips.png?branch=hydro-devel)](https://travis-ci.org/fsuarez6/grips)


## Installation

### Basic Requirements

1. Install [ROS Hydro](http://wiki.ros.org/hydro/Installation/Ubuntu) (**Desktop Install** Recommended)
2. Install [Gazebo 2.0](http://gazebosim.org/wiki/2.0/install)

### Additional Dependencies

If you installed succesfully ROS Hydro now you can install this additional packages:
```
sudo apt-get install ros-hydro-moveit-full ros-hydro-gazebo-ros-pkgs ros-hydro-gazebo-ros-control ros-hydro-ros-control ros-hydro-ros-controllers ros-hydro-robot-state-publisher ros-hydro-image-pipeline ros-hydro-cmake-modules python-wstool
``` 
### Repository Installation

Go to your ROS working directory. e.g.
```
cd ~/catkin_ws/src
``` 
Use the `wstool` to install the repository
```
wstool init .
wstool merge https://raw.github.com/fsuarez6/grips/hydro-devel/grips.rosinstall
wstool update
``` 
Check for any missing dependencies using rosdep:
```
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro hydro
``` 
After installing the missing dependencies compile your ROS workspace. e.g.
```
cd ~/catkin_ws && catkin_make
``` 

### Testing Installation

Be sure to always source the appropriate ROS setup file, which for Hydro is done like so:
```
source /opt/ros/hydro/setup.bash
``` 
You might want to add that line to your `~/.bashrc`

Try any of the `.launch` files in the `grips_gazebo` package: (e.g. `cordless_drill.launch`)
```
roslaunch grips_gazebo cordless_drill.launch
``` 

---
## Changelog
### 0.1.0 (2013-10-28)
* Initial Release

---
## Roadmap
### 0.2.0 (2013-11-11)
* Include bimanual setup

---
## Tutorials
TODO
