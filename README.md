# Inverted-Pendulum-Simulation
Gazebo simulation of inverted pendulum to get started with gazebo and ros.

Refer to [this](http://gazebosim.org/tutorials?tut=ros_plugins) tutorial to create a ROS plugin for Gazebo. 
Refer to [this](http://wiki.ros.org/msg) for building and and using msg files.

To use this example, 
- create a ROS package using catkin in workspace.
- update package.xml and cmakelists.txt.
- place all files in appropriate folders.
- compile using catkin_make.
- source setup.*sh file,
>source devel/setup.bash
- run, 
>roslaunch gazebo_tutorials inverted_pendulum_sim.launch
- This should start Gazebo in paused mode and a Hello world message will be printed on the terminal.
- Start a new terminal and run rqt_plot.
- Use rqt_plot to select the variables from "inverted_pendulum_states" that needs to be printed.
- Go to the Gazebo window and click on the Play button.

To view the states in text format, start a new terminal and run,
>rosrun gazebo_tutorials gazebo_tutorial_listener

This runs a ros executable to read messages in the inverted_pendulum_states topic and displays it on the terminal.

