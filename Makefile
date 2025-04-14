build:
	colcon build

run:
	bash -c '\
		source /opt/ros/humble/setup.bash && \
		source install/setup.bash && \
		export ROS_DOMAIN_ID=1 && \
		echo "ROS_DOMAIN_ID=$$ROS_DOMAIN_ID" && \
		ros2 run robots my_program'

.PHONY: build run