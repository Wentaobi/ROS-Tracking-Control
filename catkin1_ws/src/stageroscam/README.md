stageroscam
===========

ROS Stage World for IGVC
Stage_ros on Hydro currently support camera
( for ROS Fuerte checkout the stageroscam Fuerte branch )

Tested on ROS Hydro

To use
	cd ~/catkin_ws/src
	wstool set stageroscam --git https://github.com/udmamrl/stageroscam.git
	wstool update stageroscam
	
Run it
	roslaunch stageroscam IGVC2012.launch	

The published topic

$ rostopic list
/base_pose_ground_truth
/base_scan
/camera_info
/clicked_point
/clock
/cmd_vel
/depth
/image
/initialpose
/move_base_simple/goal
/odom
/rosout
/rosout_agg
/tf
/tf_static
