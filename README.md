# ANYmal Director

![director](director_pic.png)

# Introduction

This package contains a minimal user interface for operating the ANYmal robot.

It is based upon the Director UI.

insert image showing how director and director anymal are related.

# Cloning and Building

To clone this module and its dependencies:

	mkdir -p anymal_director_ws/src
	cd anymal_director_ws
	catkin init
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
	cd src
	git clone git@github.com:ori-drs/director_anymal_b_simple.git
	source director_anymal_b_simple/scripts/clone_deps.sh
	cd ..

If you have a workspace, you can skip several steps above

Then build the software:

    source /opt/ros/melodic/setup.bash
	catkin build

It should take about 90 seconds.

# Running UI with ANYmal simulator

Launch a simulator and the Director:

    roslaunch anymal_b_navigation_sim sim.launch
    roslaunch anymal_director_ws anymal_sim.launch

The usual ANYbotics simulator will start. The robot will also be drawn in Director
as well as sensor data.

In this basic version, you can only 'square up', lie down and stand up.
The python code for these actions is located in anymaldriverpanel.py

A more complete interface is available on demand from Oxford Dynamic Robot Systems Group