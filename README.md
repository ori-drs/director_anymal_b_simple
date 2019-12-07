# ANYmal Director

![director](director_pic.png)

# Introduction

This package contains a minimal user interface for operating the ANYmal robot. It builds upon and configures the core [Director UI](https://github.com/ori-drs/director) to be specific to ANYmal e.g. giving commands to the robot and drawing sensor signals from the robot.

View demo video with ANYmal simulator:

[![Demo video](https://img.youtube.com/vi/ZX53VhNcAuA/0.jpg)](https://www.youtube.com/watch?v=ZX53VhNcAuA)

Director is a flexible, configurable robot user interface which is configured using Python. Development can be done without needing to recompile, which enables rapid integration. It is organised as follows:

![director](director_overview.png)

# Cloning and Building

Please refer to the main Director instructions about dependencies and features. This software targets Ubuntu 18.04 and ROS Melodic.

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

This should take about 90 seconds to compile.

Currently this environment variable is also needed:

	export DRC_BASE="/src/anymal_director_ws/src/director_anymal_b_simple/"

# Running UI with ANYmal simulator

Launch a Gazebo simulator and the Director:

    roslaunch anymal_b_navigation_sim sim.launch
    roslaunch director_anymal_b_simple anymal_sim.launch

The usual ANYbotics ANYmal simulator will start. The robot will also be drawn in Director, as well as LIDAR and depth camera sensor data. See the video mentioned above.

In this basic version, you can only command the robot to 'square up', lie down and stand up.
There are quick key bindings to change view e.g. 'r' reset. 't' topview.

The python code for these actions is located in anymaldriverpanel.py. Start there for further development.

# Further Development

A more complete interface is available on request from Oxford Dynamic Robot Systems Group. It includes all the UI components which are disabled
in the version provided here and works in simulation and on the real robot. In addition to the core Director features, features specific to ANYmal Director including:

* An intuitive teleop interface with point-to-point pose control while trotting or walking
* An interactive Python interface to the ANYmal API
* Task execution and scheduling for autonomous missions

To make the full use of this software requires joining the [ANYmal Research Community](https://www.anymal-research.org/).

ANYbotics provide excellent control software, software releases and support for ANYmal within the ANYmal Research Community.
