# Steps for having independednt control of each bot
* Launch the multi_tb3_simulation_launch.py in the bringup folder
* Open a new terminal and write command $
ros2 run turtlebot3_example turtlebot3_position_control --ros args -r __ns:=/robot<n>
* Where n is the robot you want to control it.
* Proceed with step 2 again to control position for any other robot
