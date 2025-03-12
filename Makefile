build:
	colcon build

run: build
	$(source install/setup.bash)
	export ROS_DOMAIN_ID=1
	ros2 run MyProject my_program