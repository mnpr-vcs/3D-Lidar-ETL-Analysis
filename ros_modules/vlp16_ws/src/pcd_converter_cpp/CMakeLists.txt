cmake_minimum_required(VERSION 3.5)
project(pcd_converter_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# create executable
add_executable(convert_node src/pcd_converter.cpp)
ament_target_dependencies(convert_node rclcpp)

# install inside ws
install( TARGETS
  convert_node
  DESTINATION lib/${PROJECT_NAME}

)

ament_package()
