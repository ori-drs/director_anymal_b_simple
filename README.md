# ANYmal Director

![director](director_pic.png)

# Introduction

This package contains a minimal user interface for operating the ANYmal robot. It configures the core [Director UI](https://github.com/ori-drs/director) to be specific to ANYmal e.g. giving commands and receiving signals from the robot.

![director](director_overview.png)

Director is a flexible, configurable robot interface configured using Python, without needing to recompile which enables rapid integration.

# Cloning and Building

Please refer to the main Director instructions about dependencies and features.

To clone this module and its dependencies into a ROS workspace:

	mkdir -p anymal_director_ws/src
	cd anymal_director_ws
	catkin init
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
	cd src
	git clone git@github.com:ori-drs/director_anymal_b_simple.git
	source director_anymal_b_simple/scripts/clone_deps.sh
	cd ..

If you have a workspace, you can skip several of the steps above.

To build the software:

    source /opt/ros/melodic/setup.bash
	catkin build
	source devel/setup.bash

This should take about 90 seconds.

# Running UI with ANYmal simulator

Launch a Gazebo simulator and the Director:

    roslaunch anymal_b_navigation_sim sim.launch
    roslaunch director_anymal_b_simple anymal_sim.launch

The usual ANYbotics ANYmal simulator will start. The robot will also be drawn in Director, as well as LIDAR and depth camera sensor data.

In this basic version, you can only 'square up', lie down and stand up.
The python code for these actions is located in anymaldriverpanel.py

A more complete interface is available on request from Oxford Dynamic Robot Systems Group. It includes all the components which are disabled
in this version and works in simulation and the real robot. In addition to the core director features, features specific to ANYmal include:

* Point-to-point pose control while trotting
* interactive Python interface to ANYmal API
* A task execution interface

To make the full use of this software requires joining the [ANYmal Research Community](https://www.anymal-research.org/).

ANYbotics provide excellent controls software, versioning and support with ANYmal within the ANYmal Research Community.