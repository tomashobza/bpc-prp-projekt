build:
	colcon build

run: build
	$(source /opt/ros/humble/setup.bash)
	$(source install/setup.bash)
	$(export ROS_DOMAIN_ID=1)
	echo $(ROS_DOMAIN_ID)
	ros2 run MyProject my_program

.PHONY: build run