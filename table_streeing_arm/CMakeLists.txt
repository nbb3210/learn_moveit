cmake_minimum_required(VERSION 3.8)

project(table_streeing_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install directories
install(DIRECTORY
	launch
	urdf
	meshes
	config
	DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
install(PROGRAMS
	launch/display.launch.py
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
