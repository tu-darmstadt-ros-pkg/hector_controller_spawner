cmake_minimum_required(VERSION 3.8)
project(hector_controller_spawner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hector_ros2_utils REQUIRED)

add_library(hector_controller_spawner_lib SHARED
            src/hector_controller_spawner.cpp)

target_include_directories(
  hector_controller_spawner_lib
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(hector_controller_spawner_lib rclcpp std_msgs
                          controller_manager_msgs hector_ros2_utils)

add_executable(hector_controller_spawner src/hector_controller_spawner_main.cpp)
target_link_libraries(hector_controller_spawner hector_controller_spawner_lib)
ament_target_dependencies(hector_controller_spawner rclcpp std_msgs
                          controller_manager_msgs hector_ros2_utils)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # for ROS 2 launch‐testing
  find_package(launch_testing_ament_cmake REQUIRED)
  # This will install and run your test/controller_spawner_launch.py
  add_launch_test(test/test_spawner_basic.test.py TIMEOUT 360)
  add_launch_test(test/test_spawner_twice.test.py TIMEOUT 360)
  add_launch_test(test/test_spawner_estop.test.py TIMEOUT 360)
  add_launch_test(test/test_spawner_chain_detection.test.py TIMEOUT 360)
  add_launch_test(test/test_spawner_multiple_chained.test.py TIMEOUT 360)
  add_launch_test(test/test_spawner_start_partial_chain.test.py TIMEOUT 360)
  add_launch_test(test/test_spawner_gives_up.test.py TIMEOUT 360)
endif()

install(
  TARGETS hector_controller_spawner_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(TARGETS hector_controller_spawner DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(
  DIRECTORY config launch test
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs controller_manager_msgs
                          hector_ros2_utils)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
