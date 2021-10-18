# exploration_robot

## Environment

* Ubuntu __18.04__
* ROS __Melodic__
* Python
  * Version 2.7
  * Default with ROS Melodic

```bash
# Install ROS Melodic desktop full
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt install ros-melodic-desktop-full

sudo rosdep init

rosdep update

sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential python-pip python-catkin-tools

# Install ROS packages and such for assignment 1

sudo apt install ros-melodic-ros-tutorials ros-melodic-turtlebot3 ros-melodic-turtlebot3-simulations ros-melodic-navigation libspatialindex-dev libqt4-dev

sudo apt install ros-melodic-rqt ros-melodic-rqt-common-plugins ros-melodic-turtlesim

sudo apt install ros-melodic-turtle-tf2 ros-melodic-tf2-tools ros-melodic-tf

pip install rtree sklearn
```

[Source ROS](https://wiki.ros.org/melodic/Installation/Ubuntu#melodic.2BAC8-Installation.2BAC8-DebEnvironment.Environment_setup):

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

[Create a ROS workspace](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment):

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Project setup

```bash
cd ~/catkin_ws/
wstool init src
cd ~/catkin_ws/src
wstool set -y irob_assignment_1 --git https://github.com/danielduberg/irob_assignment_1.git -v master
wstool set -y hector_slam --git https://github.com/tu-darmstadt-ros-pkg/hector_slam.git -v melodic-devel
wstool update
cd ~/catkin_ws
# This makes sure we compile in release mode (which means that the compiler optimizes the code)
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
source ~/.bashrc
```

### Description

Help a [TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) robot explore an unknown environment. The robot is called Burger and you can see a picture of Burger below.

![TurtleBot3 Burger](images/turtlebot3_burger.png "TurtleBot3 Burger. Image taken from: http://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/#data-of-turtlebot3-burger")

Image taken from: [http://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/#data-of-turtlebot3-burger](http://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/#data-of-turtlebot3-burger)

The ability to perform autonomous exploration is essential for an autonomous system operating in unstructured or unknown environments where it is hard or even impossible to describe the environment beforehand.

#### System description

An _exploration_ node, based on [receding horizon "next-best-view" (RH-NBV)](https://ieeexplore.ieee.org/abstract/document/7487281), is prepared. A _collision avoidance_ node, based on [the obstacle-restriction method (ORM)](https://ieeexplore.ieee.org/abstract/document/1545546) and [pure pursuit](https://apps.dtic.mil/docs/citations/ADA255524) (for smooth control), that ensures safe path following, is used. In order for Burger to localize herself/himself you will also run a _SLAM_ node, based on [Hector SLAM](https://wiki.ros.org/hector_slam), which does mapping and localization. We will also make use of a [_costmap_2d_](https://wiki.ros.org/costmap_2d) node in order to make the exploration and collision avoidance simpler. The simulator we use is called [Gazebo](http://gazebosim.org/) and it is a popular simulator when working with ROS. Lastly, we will use a [_robot_state_publisher_](https://wiki.ros.org/robot_state_publisher) node to get the necessary transformations.

We create a _controller_ node that is using the exploration node and the collision avoidance node in order to move Burger around in the environment.

#### Run the simulator

Open three terminals.

In the first terminal you should start the ROS Master:

```bash
roscore
```

In the second terminal you should launch the file `simulator.launch` inside `irob_assignment_1/launch` like this:

```bash
roslaunch irob_assignment_1 simulator.launch
```

And in the third terminal launch the file `start.launch` inside `irob_assignment_1/launch` like this:

```bash
roslaunch irob_assignment_1 start.launch
```

In the main view of the RViz window you can see a small Turtlebot3 Burger robot in the middle of the white area. The white area of the map is called _free space_, it is space where the robot knows there is nothing. The large gray area is _unknown space_, it is space that the robot knowns nothing about. It can be either _free space_ or _occupied space_. _Occupied space_ is the cerise colored space. The cyan colored space is called _C-space_, it is space that are a distance from the _occupied space_ such that the robot would collied with the _occupied space_ if it would move into it. Our job is to help Burger explore as much of the _unknown space_ as possible.

#### Simple approach

If you go into the folder `irob_assignment_1/scripts` you will see a file called `controller.py`. Here we have made a skeleton for the controller node that you should write. So create your controller node in that file. When you are done you can test your controller by running:

```bash
rosrun irob_assignment_1 controller.py
```

If your code does not work yet, or you want to restart you simply have to close down your node and `start.launch` by going to the terminal where you started `start.launch` and press `CTRL+C`. Thereafter you launch the `start.launch` again:

```bash
roslaunch irob_assignment_1 start.launch
```

Together with your node in a seperate terminal:

```bash
rosrun irob_assignment_1 controller.py
```

__Pseudocode__ for the assignment:

```python
# Init stuff
while True:
    path, gain = get_path_from_action_server()
    if path is empty:
        exit() # Done
    while path is not empty:
      path, setpoint = get_updated_path_and_setpoint_from_service(path)
      setpoint_transformed = transform_setpoint_to_robot_frame(setpoint)
      publish(setpoint_transformed)
      sleep()
```

#### OPTIONAL: Callback based approach

If you did the mini-project using the simple approach you will notice that the exploration is quite slow and that Burger is just standing still a lot. This is because we are not using Actionlib to it's full potential. We are simply using the action server as a service.

To utilize Actionlib fully we have to use the _callback based action client_.

The difference from the none callback based action client is that we specify three callback functions when calling send_goal:

* `active_cb`: This will be called as soon as the goal has been sent to the server.
* `feedback_cb`: This will be called every time the server sends feedback to the client.
* `done_cb`: This will be called when the server is done with the goal, when the client cancel the goal, or an error happens during processing of the goal.

As you can see we also do not `wait_for_result()` or `get_result()` since we will get the feedback or result as soon as it is available in the `feedback_cb` or `done_cb` callback, respectively.

To run it you type in the terminal:

```bash
rosrun irob_assignment_1 controller_feedback.py
```