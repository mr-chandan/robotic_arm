cmake_minimum_required(VERSION 3.8)
project(robotic_arm_hw)

# Set C++ standard to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add compiler warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)  # ament_cmake MUST be first
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Find serial library
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)

# Create the library
add_library(
  robotic_arm_hw_lib
  src/robotic_arm_hw.cpp
)

# Add include directories
target_include_directories(
  robotic_arm_hw_lib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SERIAL_INCLUDE_DIRS}
)

# Add dependencies
ament_target_dependencies(
  robotic_arm_hw_lib
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Link against serial library
target_link_libraries(
  robotic_arm_hw_lib
  ${SERIAL_LIBRARIES}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the library
install(
  TARGETS robotic_arm_hw_lib
  EXPORT robotic_arm_hw_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install launch files if any
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(robotic_arm_hw_lib)
ament_export_targets(robotic_arm_hw_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Testing configuration
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Register the package
ament_package()