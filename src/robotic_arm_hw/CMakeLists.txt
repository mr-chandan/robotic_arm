cmake_minimum_required(VERSION 3.16)
project(robotic_arm_hw LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)

# Explicitly set include directory for serial
if(NOT "${SERIAL_INCLUDE_DIRS}" STREQUAL "")
  message(STATUS "Using SERIAL_INCLUDE_DIRS from pkg-config: ${SERIAL_INCLUDE_DIRS}")
else()
  message(STATUS "SERIAL_INCLUDE_DIRS was empty, setting to /usr/include/serial")
  set(SERIAL_INCLUDE_DIRS "/usr/include/serial")
endif()


# Specify the required version of ros2_control
find_package(controller_manager REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  robotic_arm_hw
  SHARED
  src/robotic_arm_hw.cpp
)

target_compile_features(robotic_arm_hw PUBLIC cxx_std_17)

target_include_directories(robotic_arm_hw PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/robotic_arm_hw>
  ${SERIAL_INCLUDE_DIRS}
)

ament_target_dependencies(
  robotic_arm_hw PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(
  robotic_arm_hw
  PUBLIC
  ${SERIAL_LIBRARIES}
)


# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface robotic_arm_hw.xml)

# INSTALL
install(
  DIRECTORY include/robotic_arm_hw
  DESTINATION include/robotic_arm_hw
)

install(
  FILES robotic_arm_hw.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS robotic_arm_hw
  EXPORT export_robotic_arm_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Remove or Add a test file
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Add this line if you do have a test file
  # ament_add_pytest_test(test_example test/test_hardware.py) #Example test
  # Add this line if you want to remove the test case
  # message("No testing for this package")
endif()

## EXPORTS
ament_export_targets(export_robotic_arm_hw HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()