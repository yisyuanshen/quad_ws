# When input make clean, execute this
clean:
	@echo "\nCleaning ROS 2 workspace...\n"
	rm -rf build install log
	@echo "\nDone.\n"

# When input make build, execute this
build:
	colcon build --symlink-install