cmake_minimum_required(VERSION 3.8)
project(controller_orchestrator)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_CXX_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# -------------------------------------------------------------------
# 1) Build the controller_orchestrator library
# -------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED src/controller_orchestrator.cpp
                                   src/switch_plan.cpp)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS} ${controller_manager_msgs_TARGETS}
         ${rcl_interfaces_TARGETS})

# -------------------------------------------------------------------
# 3) Install rules
# -------------------------------------------------------------------
# Export the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} # libraries themselves go into lib;
                                          # executables into lib/<pkg>
)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Optionally install config / launch directories
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# Export dependencies, include dirs, and targets
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs controller_manager_msgs)
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(hector_testing_utils REQUIRED)
  find_package(lifecycle_msgs REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(controller_orchestrator_test
                  test/test_controller_orchestrator.cpp TIMEOUT 120)
  target_link_libraries(
    controller_orchestrator_test
    ${PROJECT_NAME}
    ${controller_manager_msgs_TARGETS}
    hector_testing_utils::hector_testing_utils
    ${lifecycle_msgs_TARGETS}
    rclcpp::rclcpp)
endif()

ament_package()
