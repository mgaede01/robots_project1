xterm -hold roscore
xterm -hold roslaunch turtlebot_gazebo turtlebot_world.launch
REM insert
REM room.model
xterm -hold roslaunch kobuki_keyop keyop.launch
xterm -hold roslaunch turtlebot_rviz_launchers view_robot.launch
xterm -hold rosrun rqt_graph rqt_graph
cd ~/catkin_ws
catkin_make
source devel/setup.bash
cd src
REM rosrun my_project1 my_project1_scan_node
