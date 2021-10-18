# exploration_robot

## Environment

* Ubuntu __18.04__
* ROS __Melodic__
* Python
  * Version 2.7
  * Default with ROS Melodic

```bash
# Install ROS Melodic desktop full
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full
$ sudo rosdep init
$ rosdep update
$ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential python-pip python-catkin-tools

# Install ROS packages and such
$ sudo apt install ros-melodic-ros-tutorials ros-melodic-turtlebot3 ros-melodic-turtlebot3-simulations ros-melodic-navigation libspatialindex-dev libqt4-dev
$ sudo apt install ros-melodic-rqt ros-melodic-rqt-common-plugins ros-melodic-turtlesim
$ sudo apt install ros-melodic-turtle-tf2 ros-melodic-tf2-tools ros-melodic-tf
$ pip install rtree sklearn
```

[Source ROS](https://wiki.ros.org/melodic/Installation/Ubuntu#melodic.2BAC8-Installation.2BAC8-DebEnvironment.Environment_setup):

```bash
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

[Create a ROS workspace](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment):

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

## Project setup

```bash
$ cd ~/catkin_ws/
$ wstool init src
$ cd ~/catkin_ws/src
$ wstool set -y irob_assignment_1 --git https://github.com/danielduberg/irob_assignment_1.git -v master
$ wstool set -y hector_slam --git https://github.com/tu-darmstadt-ros-pkg/hector_slam.git -v melodic-devel
$ wstool update
$ cd ~/catkin_ws
# This makes sure we compile in release mode (which means that the compiler optimizes the code)
$ catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
$ source ~/.bashrc
```

## Description

Help a [TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) robot explore an unknown environment. The robot is called Burger and you can see a picture of Burger below.

![TurtleBot3 Burger](images/turtlebot3_burger.png "TurtleBot3 Burger. Image taken from: http://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/#data-of-turtlebot3-burger")

### System description

* `exploration node`: based on [receding horizon "next-best-view" (RH-NBV)](https://ieeexplore.ieee.org/abstract/document/7487281) with a RRT tree.
* `collision avoidance node`: based on [the obstacle-restriction method (ORM)](https://ieeexplore.ieee.org/abstract/document/1545546) and [pure pursuit](https://apps.dtic.mil/docs/citations/ADA255524) (for smooth control), that ensures safe path following.
* `SLAM node`: for Burger to localize herself/himself, based on [Hector SLAM](https://wiki.ros.org/hector_slam), which does mapping and localization.
* `costmap_2d node`: [_costmap_2d_](https://wiki.ros.org/costmap_2d) node in order to make the exploration and collision avoidance simpler.
* `robot_state_publisher node`: [_robot_state_publisher_](https://wiki.ros.org/robot_state_publisher) node to get the necessary transformations.
* `controller node`: using the exploration node and the collision avoidance node in order to move Burger around in the environment.

### Run the simulator

Open four terminals.

In the first terminal you should start the ROS Master:

```bash
$ roscore
```

In the second terminal you should launch the file `simulator.launch` inside `irob_assignment_1/launch` like this:

```bash
$ roslaunch irob_assignment_1 simulator.launch
```

And in the third terminal launch the file `start.launch` inside `irob_assignment_1/launch` like this:

```bash
$ roslaunch irob_assignment_1 start.launch
```

In the fourth terminal run the controller.
#### Simple approach

```bash
$ rosrun irob_assignment_1 controller.py
```

#### Callback based approach

```bash
$ rosrun irob_assignment_1 controller_feedback.py
```