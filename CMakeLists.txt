cmake_minimum_required(VERSION 3.8)
project(d2w_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include cpp 'include' directory
include_directories(include)

# Add internal libs folder
add_subdirectory(lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# create cpp executable
add_executable(comms src/comms.cpp)
ament_target_dependencies(comms rclcpp sensor_msgs std_msgs)
target_link_libraries(comms mpu9250sensor linux_i2c_communicator drive_comms)
target_include_directories(
  comms PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)

# install python modules
ament_python_install_package(${PROJECT_NAME})

# install python executables
install(PROGRAMS 
  scripts/teleop_twist_keyboard.py
  scripts/vel_publisher.py
  scripts/ref_publisher.py
  scripts/vel_drive.py
  scripts/control_drive.py
  DESTINATION lib/${PROJECT_NAME}
)
# install cpp executables
install(TARGETS comms DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME}/)

ament_package()
