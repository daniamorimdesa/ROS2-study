cmake_minimum_required(VERSION 3.8)
project(final_project_level3_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)

# C++ Lifecycle Component
add_library(turtle_controller_component SHARED
  src/turtle_controller_component.cpp)
target_include_directories(turtle_controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(turtle_controller_component PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(turtle_controller_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  rclcpp_components
  turtlesim
  geometry_msgs
  my_robot_interfaces)

# Register component
rclcpp_components_register_node(turtle_controller_component
  PLUGIN "final_project::TurtleController"
  EXECUTABLE turtle_controller_cpp)

# Install C++ library
install(TARGETS turtle_controller_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
